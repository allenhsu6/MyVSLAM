//
// Created by xu on 2019/12/13.
//

#include "visual_odometry.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include <Config.h>

namespace myslam
{
// 初始化所有的属性
VisualOdometry::VisualOdometry():
    state_ (INITIALIZING), ref_(nullptr), curr_(nullptr),
    map_(new Map), num_lost_(0), num_inliers_(0)
{
    num_of_features_    = Config::get<int>("number_of_features");
    scale_factor_       = Config::get<double>("scale_factor");
    level_pyramid_      = Config::get<int>("level_pyramid");
    match_ratio_        = Config::get<float>("match_ratio");
    max_num_lost_       = Config::get<float>("max_num_lost");
    min_inliers_        = Config::get<int>("min_inliers");
    key_frame_min_rot   = Config::get<double>("keyframe_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_translation");
    orb_ = cv::ORB::create(num_of_features_, scale_factor_,level_pyramid_);
}

VisualOdometry::~VisualOdometry() {

}
//  这个函数很关键，组织了整个状态的判断，穿起来下面所有的函数
// 所以写代码的时候，先搭建框架，然后再组织细节。
bool VisualOdometry::addFrame(Frame::Ptr frame) {
    switch (state_) {
        case INITIALIZING:      // 将frame设置为参考帧
        {
            state_ = OK;
            curr_ = ref_ = frame;
            map_->insertKeyFrame(frame);
            // 从第一帧中提取特征点
            extractKeyPoints();
            computeDescriptors();
            // 计算参考帧中特征点的三维位置
            setRef3DPoints();
            break;
        }
        case OK:
        {
            curr_ = frame;
            // map_->insertKeyFrame(frame);

            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            // 这里直接开始pnp 意味着ref的深度已经设置好了
            poseEstimationPnP();
            if (checkEstimatedPose()){
                curr_->T_c_w_ = T_c_r_estimated_ * ref_->T_c_w_;
                ref_ = curr_;
                // 匹配完成之后，如果估计的位姿可以接受，我们就可以更新ref为当前帧
                // 顺便把描述子也搬家过来
                setRef3DPoints();
                num_lost_ = 0;
                if (checkKeyFrame()){
                    addKeyFrame();
                }
            }else{
                num_lost_++;
                if (num_lost_ > max_num_lost_){
                    state_ = LOST;
                }
                return false;
            }
            break;
        }
        case LOST:
        {
            cout << "vo has lost. " << endl;
            break;
        }
    }
    return true;
}

// 从image color中提取出来对应的关键点，放在keypoints序列中
void VisualOdometry::extractKeyPoints() {
    orb_->detect(curr_->color_, keypoints_curr_);
}

// 计算描述子的部分，可以替换成课堂中学到的部分
void VisualOdometry::computeDescriptors() {
    orb_->compute(curr_->color_,keypoints_curr_,descriptors_curr_);
}

void VisualOdometry::featureMatching(){
// using OpenCV‘s brute force match
// we now have descriptor of curr and ref
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher (cv::NORM_HAMMING);
    matcher.match(descriptors_ref_,descriptors_curr_,matches);
    // select the best matches, 注意这种比较写法
    float min_dis = std::min_element(
            matches.begin(), matches.end(),
            [] (const cv::DMatch& m1, const cv::DMatch& m2){
                return m1.distance < m2.distance;
            }
            )->distance;
    feature_matches_.clear();
    for (auto m : matches){
        if (m.distance < max<float>(min_dis * match_ratio_, 30)){
            feature_matches_.push_back(m);
        }
    }
    cout << "good matches: " << feature_matches_.size() << endl;
}

void VisualOdometry::setRef3DPoints() {
    pts_3d_ref.clear();
    descriptors_ref_ = Mat();
    for (size_t i = 0; i < keypoints_curr_.size(); ++i) {
        double d = ref_->findDepth(keypoints_curr_[i]);
        if(d > 0){
            Vector3d p_cam = ref_->camera_->pixel2camera(
                    Vector2d(keypoints_curr_[i].pt.x, keypoints_curr_[i].pt.y),d);
            pts_3d_ref.push_back(cv::Point3f(p_cam[0],p_cam[1], p_cam[2]));
            // 每个keypoint都有对应的row(i)做其描述子
            descriptors_ref_.push_back(descriptors_curr_.row(i));
        }
    }
}

void VisualOdometry::poseEstimationPnP() {
    // 3D-2D
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (auto m: feature_matches_) {
        pts3d.push_back(pts_3d_ref[m.queryIdx]);
        pts2d.push_back(keypoints_curr_[m.trainIdx].pt);
    }
    // 使用opencv自带的pnpransac做位姿估计
    Mat K = (cv::Mat_<double>(3,3)<<
        ref_->camera_->fx_, 0, ref_->camera_->cx_,
        0, ref_->camera_->fy_, ref_->camera_->cy_,
        0, 0 ,1);
    Mat rvec, tvec, inliers;

    cv::solvePnPRansac(pts3d,pts2d,K,Mat(),rvec,tvec,false,100, 4,0.99,inliers);
    num_inliers_ = inliers.rows;
    cout << "pnp inliers: " << num_inliers_ << endl;
    T_c_r_estimated_ = SE3(
            SO3(rvec.at<double>(0,0),rvec.at<double>(1,0), rvec.at<double>(2,0)),
            Vector3d(tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0))
            );
}


void VisualOdometry::addKeyFrame() {
    cout << "add a keyframe" << endl;
    map_->insertKeyFrame(curr_);
}

bool VisualOdometry::checkEstimatedPose() {
    if(num_inliers_ < min_inliers_){
        cout << "reject because inliner is too small: " << num_inliers_ << endl;
        return false;
    }
    Sophus::Vector6d d = T_c_r_estimated_.log();
    if(d.norm() > 5){
        cout << "reject because motion is too large: " << d.norm() << endl;
    }
    return true;
}

bool VisualOdometry::checkKeyFrame() {
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head(3);
    Vector3d rot = d.tail(3);
    // cout << "位姿： trans: " << trans.transpose() << "  rot: "<< rot.transpose()<< endl;
    return rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans;
}

}
