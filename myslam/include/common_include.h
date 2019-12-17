//
// Created by xu on 2019/12/12.
//

#ifndef MYSLAM_COMMON_INCLUDE_H
#define MYSLAM_COMMON_INCLUDE_H

#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SE3;
using Sophus::SO3;

#include <opencv2/core/core.hpp>
using cv::Mat;

#include <vector>
#include <list>
#include <string>
#include <iostream>
#include <unordered_map>
#include <map>
#include <memory>

using namespace std;

#endif //MYSLAM_COMMON_INCLUDE_H
