//
// Created by xu on 2019/12/13.
//

#ifndef MYSLAM_VISUAL_ODOMETRY_H
#define MYSLAM_VISUAL_ODOMETRY_H

#include <common_include.h>
#include <Map.h>
#include <opencv2/features2d/features2d.hpp>

namespace myslam
{
class VisualOdometry
{
public:
    typedef shared_ptr<VisualOdometry> Ptr;
    enum VOState{
        INITIALIZING = -1,
        OK = 0,
        LOST
    };

    VOState     state_;         // VO状态
    Map::Ptr    map_;           //
    Frame::Ptr  curr_;
    Frame::Ptr  ref_;

    cv::Ptr<cv::ORB>        orb_;  // orb detector and computer
    vector<cv::Point3f>     pts_3d_ref; // ref's 3d
    vector<cv::KeyPoint>    keypoints_curr_;
    Mat                     descriptors_curr_;
    Mat                     descriptors_ref_;
    vector<cv::DMatch>      feature_matches_;

    SE3 T_c_r_estimated_;
    int num_inliers_;
    int num_lost_;

    // paramters

    int num_of_features_;
    double scale_factor_;   // scale in image pyramid
    int level_pyramid_;
    float match_ratio_;     // opencv's brute force match paramters
    int max_num_lost_;      // 允许最大丢失帧数
    int min_inliers_;

    double key_frame_min_rot;   // minimal rotation of two key-frame
    double key_frame_min_trans; // minimal translation of two key-frame

    // function
public:
    VisualOdometry();
    ~VisualOdometry();

    bool addFrame(Frame::Ptr frame);    // add a new frame

protected:
    // inner operation
    void extractKeyPoints();
    void computeDescriptors();
    void featureMatching();
    void poseEstimationPnP();
    void setRef3DPoints();

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

};

}


#endif //MYSLAM_VISUAL_ODOMETRY_H
