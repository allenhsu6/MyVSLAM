//
// Created by xu on 2019/12/17.
//

#include "g2o_types.h"
using namespace g2o;

namespace myslam{

// g2o中的定义是旋转在前，平移在后 Sophus，Eigen是平移在前
void EdgeProjectXYZ2UV::computeError(){
    // pose
    const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    // landmark
    const VertexSBAPointXYZ* v2 = static_cast<const VertexSBAPointXYZ*>(_vertices[0]);
    Vector2d obs(_measurement);
    // w误差等于 观测减去预测
    _error = obs - camera_->camera2pixel(v1->estimate().map(v2->estimate()));
}

void EdgeProjectXYZ2UV::linearizeOplus(){
    const VertexSE3Expmap* vj = static_cast<const VertexSE3Expmap*>(_vertices[1]);
    SE3Quat T(vj->estimate());
    // landmark
    const VertexSBAPointXYZ *vi = static_cast<const VertexSBAPointXYZ *>(_vertices[0]);
    Vector3d xyz = vi->estimate();
    Vector3d xyz_cam = T.map(xyz);  // 相机坐标
    double x = xyz_cam[0];
    double y = xyz_cam[1];
    double z = xyz_cam[2];
    double z_2 = z * z;
    // tmp
    Eigen::Matrix<double, 2, 3, Eigen::ColMajor> tmp;
    tmp(0,0) = camera_->fx_;
    tmp(0,1) = 0;
    tmp(0,2) = -camera_->fx_ * x/z;
    tmp(1,0) = 0;
    tmp(1,1) = camera_->fy_;
    tmp(1,1) = -camera_->fy_ * y/z;

    // vertices[0]是landmark, javobianxi表示error对第一个点landmark求导 [2*3]
    _jacobianOplusXi = (-1/z) * tmp * T.rotation().toRotationMatrix();

    // vertices[1]是六个维度的位姿 [2*6]的矩阵
    _jacobianOplusXj ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXj ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXj ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXj ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXj ( 0,4 ) = 0;
    _jacobianOplusXj ( 0,5 ) = x/z_2 * camera_->fx_;

    _jacobianOplusXj ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXj ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXj ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXj ( 1,3 ) = 0;
    _jacobianOplusXj ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXj ( 1,5 ) = y/z_2 *camera_->fy_;
}


void EdgeProjectXYZRGBDPoseOnly::computeError() {
    const g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    SE3Quat T(pose->estimate());
    _error = _measurement - T.map(point_);
}


// 旋转在前，平移在后
void  EdgeProjectXYZRGBDPoseOnly::linearizeOplus(){
    const g2o::VertexSE3Expmap* pose = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
    SE3Quat T(pose->estimate());
    Vector3d xyz = T.map(point_);
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    _jacobianOplusXi(0,0) = 0;
    _jacobianOplusXi(0,1) = -z;
    _jacobianOplusXi(0,2) = y;
    _jacobianOplusXi(0,3) = -1;
    _jacobianOplusXi(0,4) = 0;
    _jacobianOplusXi(0,5) = 0;

    _jacobianOplusXi(1,0) = z;
    _jacobianOplusXi(1,1) = 0;
    _jacobianOplusXi(1,2) = -x;
    _jacobianOplusXi(1,3) = 0;
    _jacobianOplusXi(1,4) = -1;
    _jacobianOplusXi(1,5) = 0;

    _jacobianOplusXi(2,0) = -y;
    _jacobianOplusXi(2,1) = x;
    _jacobianOplusXi(2,2) = 0;
    _jacobianOplusXi(2,3) = 0;
    _jacobianOplusXi(2,4) = 0;
    _jacobianOplusXi(2,5) = -1;
}


void EdgeProjectXYZ2UVPoseOnly::computeError(){
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    SE3Quat T(pose->estimate());
    _error = _measurement - camera_->camera2pixel
            (T.map(point_));
}
void EdgeProjectXYZ2UVPoseOnly::linearizeOplus(){
    const g2o::VertexSE3Expmap* pose = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
    // 位姿T
    SE3Quat T(pose->estimate());
    // 第二帧的x , y z
    Vector3d xyz = T.map(point_);
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];
    double z_2 = z * z;

    _jacobianOplusXi ( 0,0 ) =  x*y/z_2 *camera_->fx_;
    _jacobianOplusXi ( 0,1 ) = - ( 1+ ( x*x/z_2 ) ) *camera_->fx_;
    _jacobianOplusXi ( 0,2 ) = y/z * camera_->fx_;
    _jacobianOplusXi ( 0,3 ) = -1./z * camera_->fx_;
    _jacobianOplusXi ( 0,4 ) = 0;
    _jacobianOplusXi ( 0,5 ) = x/z_2 * camera_->fx_;

    _jacobianOplusXi ( 1,0 ) = ( 1+y*y/z_2 ) *camera_->fy_;
    _jacobianOplusXi ( 1,1 ) = -x*y/z_2 *camera_->fy_;
    _jacobianOplusXi ( 1,2 ) = -x/z *camera_->fy_;
    _jacobianOplusXi ( 1,3 ) = 0;
    _jacobianOplusXi ( 1,4 ) = -1./z *camera_->fy_;
    _jacobianOplusXi ( 1,5 ) = y/z_2 *camera_->fy_;
}



}


