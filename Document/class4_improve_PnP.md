## 改进SLAM前端：第一版

在上一讲中，我们实现了帧帧之间的里程计。无论是速度还是精度，效果都不够理想。

出现这种情况的**原因**有：

1. 位姿估计的时候，我们使用RanSAC求出的PnP解。其中RanSAC只是少量的随机点来计算PnP，虽然能够确定inlier，但是方法太容易受到噪声影响。 

2. RGBD相机的深度图存在误差，尤其是深度过近或过远的地方。而且特征点提取的往往是角点，深度通常是不准确的。
3. 只使用帧间的匹配，导致结果太依赖参考帧，如果参考帧位姿估计不准确，会出现明显漂移。同时，这种做法没有充分利用全部信息。

4. 通过对时间的统计，发现在大部分时间浪费在提取特征点与特征点上。

对应的**解决办法**：

1. 使用RANSAC解作为初始值，然后使用非线性优化求最优值。
2. 非线性优化过程中，同时将特征点一起优化。
3. 使用frame-map的匹配，同时优化地图点。
4. 使用光流/直接法做前端，减少运算量。

总得来说就是BA必须引进来。



### PnP优化

![pnp的图优化表示](/home/xu/MyVSLAM/Document/image/image-20191217152229190.png)



如果仅看相机位姿的话，我们无需优化地图点。但是为了最终考虑，我觉得这部分还是很有必要的。

每一个两帧之间组成的pnp问题，都可以是一个重投影误差构成的优化问题。

如果使用g2o这个库的话，我们需要定义一个二元边。使用g2o提供给我们的两种点的类型。



使用opencv中ransacpnp给出的结果作为初始值，开始迭代求解。



### 增加g2o自定义边

使用g2o自定义的两个定点类型：xyz与pose

需要我们根据具体情况定义不同的边：

在g2o_types.cpp 与 g2o_types.h两个文件中，我们分别定义了三种可能用到的边：



1. 常见的二元边，landmark和pose作为图的两个定点。这部分会在我们后端优化的时候用到。到时候注意使用H矩阵的稀疏性质

   ```c++
   // 二元边定义, 这部分与ch7中书上的定义完全一致
   // 在做BA批量优化的时候，注意这部分需要进行稀疏求解
   class EdgeProjectXYZ2UV: public g2o::BaseBinaryEdge<2, Vector2d, g2o::VertexSBAPointXYZ, g2o::VertexSE3Expmap>{
   public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
       virtual void computeError();
       virtual void linearizeOplus();
       virtual bool read(std::istream& in){}
       virtual bool write(std::ostream& out) const {}
   
       Camera* camera_;
   };
   ```

2. ICP问题中的一元边，这部分不算重投影误差，单纯对pose进行优化。适用于已经知道相互关联情况下的匹配问题

   ```c++
   // 自定义一元边，单纯对pose做优化, 用于ICP中的优化
   class EdgeProjectXYZRGBDPoseOnly: public g2o::BaseUnaryEdge<3, Vector3d, g2o::VertexSE3Expmap>{
   public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
       virtual void computeError();
       virtual void linearizeOplus();
       virtual bool read(std::istream& in){}
       virtual bool write(std::ostream& out) const {}
   
       Vector3d point_;
   };
   ```

   

3. 本次对pnp输出结果做的优化，这部分跟第一部分很相似，只不过我们不优化landmark，所以同样是一个一元边。

   ```c++
   // 自定义一元边，单纯对pose做优化, 使用相机参数
   class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Vector2d, g2o::VertexSE3Expmap>{
   public:
       EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
       virtual void computeError();
       virtual void linearizeOplus();
       virtual bool read(std::istream& in){}
       virtual bool write(std::ostream& out) const {}
   
       Camera* camera_;
       Vector3d point_;
   };
   ```

### 更改函数poseEstimationPnP

 如本次优化最开始所言，我们要将原来pnp输出作为初始值，然后使用非线性优化处理。具体代码实现，详见对应类中对应函数。同时为了加速，将原来的brushmatch改为flannmatch

**实现过程中的报错**

 1. **报错：** error while loading shared libraries: libg2o_core.so: cannot open shared object file: No such file o

    **解决办法：**

    ```shell
    sudo gedit /etc/ld.so.conf
    # 添加如下代码
    /usr/local/lib
    # 运行
    sudo ldconfig
    ```

2. **报错：** /usr/include/eigen3/Eigen/src/Core/MapBase.h:168: void Eigen::MapBase<Derived, 0>::checkSanity() const [with Derived = Eigen::Map<Eigen::Matrix<double, 2, 6, 0, 2, 6>, 32, Eigen::Stride<0, 0> >]: Assertion `((size_t(m_data) % (((int)1 >=  (int)internal::traits::Alignment) ? (int)1 : (int)internal::traits::Alignment)) == 0) && “data is not  aligned”’ failed.已放弃 (核心已转储)

   **问题原因：**Eigen对齐问题

   **解决办法：**

   https://blog.csdn.net/weixin_41353960/article/details/94738106

   https://blog.csdn.net/wojiushixiangshi/article/details/78356271

   使用其中一个方法： 将Cmakelists.txt中的 -march=native 删除

   最终实现正确运行。

   **启发：** 遇到这种一上来报错的问题，多半不是你的代码逻辑错误，这类行问题有很多网络资料，稍微查阅一下就好。

 3. **报错：** 未定义的引用类问题

    这种问题很常见。C++在编译为obj文件的时候，不需要函数的具体实现，只要函数原型就可以。但是在链接可执行文件的时候，必须有具体的实现。

    扩展，未定义的申明，就是没有函数原型。通常是头文件未包含。