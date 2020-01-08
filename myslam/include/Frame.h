//
// Created by xu on 2019/12/12.
// 帧的定义： 其中包括当前相机位置，对应图像上提取出的特征点。

#ifndef MYSLAM_FRAME_H
#define MYSLAM_FRAME_H

#include <common_include.h>
#include <Camera.h>

namespace myslam
{
class MapPoint;
class Frame
{
public:
    typedef shared_ptr<Frame> Ptr;
    unsigned long           id_;                 // id
    double                  time_stamp_;
    SE3                     T_c_w_;
    Camera::Ptr             camera_;            // 相机模型
    Mat                     color_, depth_;     // 彩色与灰度图

    bool                    is_key_frame_;      // 是否为关键帧

    Frame();
    Frame(long id, double time_stamp=0, SE3 T_c_w=SE3(),
            Camera::Ptr camera= nullptr, Mat color=Mat(), Mat depth=Mat());
    ~Frame();

    // 创建Frame
    static Frame::Ptr createFrame();

    // 给定点对应的深度
    double findDepth(const cv::KeyPoint& kp);

    // 相机光心
    Vector3d getCamCenter() const;

    // 判断点是否在视野之内
    bool isInFrame(const Vector3d& pt_world);
};

}

#endif //MYSLAM_FRAME_H
