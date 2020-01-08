//
// Created by xu on 2019/12/17.
// 我们实现所有在pnp中可能使用到的点的类型
//

#ifndef MYSLAM_G2O_TYPES_H
#define MYSLAM_G2O_TYPES_H

#include <common_include.h>
#include <Camera.h>         // 需要相机的信息

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

namespace myslam{
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



// 自定义一元边，单纯对pose做优化, 使用相机参数
    class EdgeProjectXYZ2UVPoseOnly: public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap >
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        virtual void computeError();
        virtual void linearizeOplus();

        virtual bool read( std::istream& in ){}
        virtual bool write(std::ostream& os) const {};

        Vector3d point_;
        Camera* camera_;
    };

}

#endif //MYSLAM_G2O_TYPES_H
