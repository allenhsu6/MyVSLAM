//
// Created by xu on 2019/12/12.
//

#include "../include/Frame.h"

namespace myslam
{
myslam::Frame::Frame(long id, double time_stamp, SE3 T_c_w, myslam::Camera::Ptr camera, Mat color, Mat depth)
:id_(id), time_stamp_(time_stamp), T_c_w_(T_c_w), camera_(camera), color_(color),depth_(depth), is_key_frame_(false)
{

}

myslam::Frame::Frame()
:id_(-1), time_stamp_(-1), camera_(nullptr), is_key_frame_(false)
{
}

myslam::Frame::~Frame() {

}

Frame::Ptr Frame::createFrame() {
    static long factory_id = 0;
    return Frame::Ptr(new Frame(factory_id++));
}

Vector3d Frame::getCamCenter() const {
    return T_c_w_.inverse().translation();
}

bool Frame::isInFrame(const Vector3d &pt_world) {
    Vector3d p_cam = camera_->world2camera(pt_world, T_c_w_);
    if (p_cam(2,0)<0){
        return false;
    }
    Vector2d pixel = camera_->camera2pixel(p_cam);
    return pixel(0,0) > 0 && pixel(1,0)>0
    && pixel(0,0) < color_.cols
    && pixel(1,0) < color_.rows;
}

double Frame::findDepth(const cv::KeyPoint &kp) {
    int x = cvRound(kp.pt.x);
    int y = cvRound(kp.pt.y);
    ushort d = depth_.ptr<ushort>(y)[x];
    if (d!=0){
        return double(d)/camera_->depth_scale_;
    } else{
        // 看看周围
        int dx[4] = {-1, 0, 1, 0};
        int dy[4] = {0, -1, 0, 1};
        for (int i = 0; i < 4; ++i) {
            d = depth_.ptr<ushort>(y+dy[i])[x+dx[i]];
            if(d != 0){
                return double(d)/camera_->depth_scale_;
            }
        }
    }
    return -1.0;
}

}


