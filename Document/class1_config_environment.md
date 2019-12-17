## 环境说明

ubuntu16.4 + eigen3 + sophus非模板库

给出对应的百度网盘链接：

如果有ros需求，可以按照如下连接进行ros安装

https://blog.csdn.net/luvalluo/article/details/78745677

### 1. eigen3安装

apt-get方式(可在终端中输入`locate eigen3`查看安装位置)。运行命令：

```shell
sudo apt-get install libeigen3-dev
```

### 2. sophus安装

解压对应的文件，然后按照cmake工程进行安装。

### 3. opencv安装

如果是安装了ros，对应的opencv目录在`/opt/ros/kinetic/include/opencv-3.3.1-dev`中

如果提示找不到opencv的话，手动设置opencv目录：

` set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev/)`

### 4. g2o安装

```shell
sudo apt-get install libsuitesparse-dev
sudo apt-get install qtdeclarative5-dev
sudo apt-get install qt5-qmake
sudo apt-get install libqglviewer-dev  //安装依赖项
cd g2o   //进入g2o文件夹
mkdir build     //创建build文件夹
cd build    //进入 build
cmake ..   //cmak编译  这个过程比较漫长
sudo make install   //安装即可
```

### 5.  pangolin安装

这部分看github对应要求的第三方，其余的部分与g2o一样。

https://github.com/stevenlovegrove/Pangolin 

