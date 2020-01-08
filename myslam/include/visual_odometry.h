//
// Created by xu on 2019/12/13.
// 最后这一版本，挺复杂的
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
    vector<cv::Point3f>     pts_3d_ref_; // ref's 3d
    vector<cv::KeyPoint>    keypoints_curr_;
    Mat                     descriptors_curr_;
    Mat                     descriptors_ref_;
    vector<cv::DMatch>      feature_matches_;
    cv::FlannBasedMatcher   matcher_flann_;     // flann matcher

    vector<MapPoint::Ptr>   match_3dpts_;           // 已经匹配好的三维空间点
    vector<int>             match_2dkp_index_;      // cur对应的3D-2D中的索引

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
    double map_point_erase_ratio_;   // remove map point ratio
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
    void optimizeMap();

    void addMapPoints();

    double getViewAngle(Frame::Ptr frame, MapPoint::Ptr point);

    void addKeyFrame();
    bool checkEstimatedPose();
    bool checkKeyFrame();

};

}


#endif //MYSLAM_VISUAL_ODOMETRY_H
