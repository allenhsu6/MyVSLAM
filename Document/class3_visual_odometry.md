## 特征点的提取与匹配

本节课程要求实现两帧之间的视觉里程计。根据输入的两帧图像，输出两帧之间的相机运动，特征点的三维坐标。

### 1. 思路

基于两帧图像做估计的里程计，我们可以使用下图做形象表示。

![image-20191213143057626](/home/xu/MyVSLAM/Document/image/class1_config_environment.md)

从t-1到t时刻，可以使用特征点匹配、光流以及直接法得到。本节课只关心相机位姿，不关心环境结构。利用特征点求出对应T后就丢弃掉该帧的特征点。我们在这里决定使用PnP特征点匹配的方法。

### 2. 算法流程

1. 对t时刻的current frame， extrame keypoint and descriptor
2. 如果系统没有初始化，该帧作为关键帧，根据深度图计算对应关键点的3D位置， return 1
3. 估计当前帧与参考帧的运动
4. 判断估计是否成功
5. 成功的话，把当前帧当作新的参考帧，return 1
6. 若失败，开始计算连续丢帧数，如果丢失超过一定帧数，设置VO为丢失，算法结束。若未超过，return 1

<后续画个算法里程图>

### 3. 实现

建立visualOdometry类

属性：

- VOstate
- map
- 两个frame： cur和ref
- 涉及到匹配的两帧对应的关键点，描述子，match结果
- 一些参数：匹配上的数量，lost的数量，估计的位姿，特征点数量，金字塔层数，scale，帧间旋转与位移的限制等等

在正常设计系统的时候，不可能一次性将所有的属性和方法都考虑清楚，是一个逐渐迭代的过程，所谓的设计方法吧。

方法：

根据上述算法流程，我们可以尽量把负责实现某个功能的代码段拆分成函数。所以设计了如下函数：

```c++
public:    
VisualOdometry();
~VisualOdometry();
bool addFrame(Frame::Ptr frame);    // add a new frame

protected:     
void extractKeyPoints();    
void computeDescriptors();    
void featureMatching();   
void poseEstimationPnP();  
void setRef3DPoints();  
void addKeyFrame();    
bool checkEstimatedPose();    
bool checkKeyFrame();
```

其中addFrame函数比较关键，与外部frame搭建桥梁。根据VO的三种状态，将下面的几个对应函数穿起来。

这样设计的好处是，当我们要升级VO系统的时候，只需要将addFrame函数做对应更改就可以，带保护的函数基本不用修改。

具体实现基本基于opencv提供的方法，详见VisualOdometry类

### 4. 测试

测试上述方法定位准确性

**准备工作：**

- 确保opencv3+viz模块
- 准备TUM数据集中任意一个，熟悉TUM数据集格式
- 在配置文件中将对应参数正确填写
- 使用测评工具evo：这部分提供链接，防止没有主次之分

**测试流程：**

读入config文件->添加相机->按照顺序添加帧->根据计算出来的帧的T更新相机位姿



**测试结果：**

这部分写使用测评工具最终的输出：

**注意事项：**

yaml文件自己生成的怎么也无法读取，会报错-49，可能有些看不见的文本格式问题。需要使用opencv先生成，然后再修改。



### 5. 感悟

做视觉相关内容的话，opencv库真的很需要熟悉。很多时候，我们需要使用opencv作出一个baase版本，然后基于其再做改进。