## 搭建SLAM框架

### 1. 关键类的撰写

- Camera
- Frame
- Map
- MapPoint
- Config

### 2. Camera类

其中主要包括camera的相关信息，其中包括相机内参、depth_scale等属性，world、camera、pixel三个坐标系之间的坐标变换

### 3. MapPoint类

属性：

- id
- pos in world
- norm 朝向
- descriptor
- 观测次数
- 被匹配次数

方法：

- 创建路标点

### 4. Frame类

属性：

- id
- camera信息
- 该帧的三维坐标
- 两张image：color and depth

方法：

- 求给定点的深度
- 相机光心（就是相机在世界坐标系下的位置）
- 判断点是否在该frame中

### 5. Map类

属性：

- mappoints集合
- frames集合

因为过程中随机访问，随时增加，删除和更新，所以使用hash表无序存储。

方法：

- 插入关键帧
- 插入路标点

### 6. config类

负责参数文件读取，程序任意地方提供参数数值。

根据上述要求，将config设计为singleton，只有一个全局对象。当设置参数文件的时候，创建对象并读取文件，程序结束时，自动销毁。

在文件读取方面，使用opencv提供打filestorage类。

属性：

- cv::FileStorage file_;

函数：

- 设置参数文件

- 模板函数get：获取任意类型的参数值

### 7. 总结

截至目前为止，我们初步定义了整个前端使用到的类。接下来我们需要在VO中实现一些功能，同时在test文件中编写一些测试用例，验证算法的正确性。

