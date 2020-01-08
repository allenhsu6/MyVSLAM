//
// Created by xu on 2019/12/13.
//

#include "visual_odometry.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <algorithm>
#include <boost/timer.hpp>
#include <Config.h>
#include <g2o_types.h>

namespace myslam
{
// 初始化所有的属性
VisualOdometry::VisualOdometry():
    state_ (INITIALIZING), ref_(nullptr), curr_(nullptr),
    map_(new Map), num_lost_(0), num_inliers_(0),matcher_flann_( new cv::flann::LshIndexParams(5,10,2))
{
    num_of_features_    = Config::get<int>("number_of_features");
    scale_factor_       = Config::get<double>("scale_factor");
    level_pyramid_      = Config::get<int>("level_pyramid");
    match_ratio_        = Config::get<float>("match_ratio");
    max_num_lost_       = Config::get<float>("max_num_lost");
    min_inliers_        = Config::get<int>("min_inliers");
    key_frame_min_rot   = Config::get<double>("keyframe_rotation");
    key_frame_min_trans = Config::get<double>("keyframe_translation");
    map_point_erase_ratio_ = Config::get<double>("map_point_erase_ratio");
    orb_ = cv::ORB::create(num_of_features_, scale_factor_,level_pyramid_);
}

VisualOdometry::~VisualOdometry() {

}

// 程序尽量设计成为面向属性的函数，不要老是传参
bool VisualOdometry::addFrame(Frame::Ptr frame) {
    switch (state_) {
        case INITIALIZING:      // 将frame设置为参考帧
        {
            state_ = OK;
            curr_ = ref_ = frame;
            extractKeyPoints();
            computeDescriptors();
            // the first frame is key frame
            addKeyFrame();
            break;
        }
        case OK:
        {
            curr_ = frame;
            // todo: 每次都改为初始值？ 如果后面不再更改ref_，下面这句话不需要
            curr_->T_c_w_ = ref_->T_c_w_;
            extractKeyPoints();
            computeDescriptors();
            featureMatching();
            poseEstimationPnP();

            if(checkEstimatedPose()){
                curr_->T_c_w_ = T_c_r_estimated_;
                optimizeMap();      // 如果觉得本次位姿估计可取
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
    boost::timer timer;
    orb_->detect(curr_->color_, keypoints_curr_);
    cout << "特征点提取耗时： " << timer.elapsed() << endl;
}

void VisualOdometry::computeDescriptors() {
    boost::timer timer;
    orb_->compute(curr_->color_,keypoints_curr_,descriptors_curr_);
    cout << "描述子计算耗时： " << timer.elapsed() << endl;
}

void VisualOdometry::featureMatching(){
// using OpenCV‘s brute force match
// we now have descriptor of curr and ref
    boost::timer timer;
    vector<cv::DMatch> matches;

    Mat desp_map;
    // 往上述的map中装载地图中可靠的描述子
    vector<MapPoint::Ptr> candidate;
    for (auto& allpoints: map_->map_points_) {
        MapPoint::Ptr& p = allpoints.second;
        // 判断是否在视野内
        if (curr_->isInFrame(p->pos_)){
            p->visible_time_ += 1;
            candidate.push_back(p);
            desp_map.push_back(p->descriptor_);
        }
    }

    matcher_flann_.match(desp_map,descriptors_curr_,matches);
    // select the best matches, 注意这种比较写法
    float min_dis = std::min_element(
            matches.begin(), matches.end(),
            [] (const cv::DMatch& m1, const cv::DMatch& m2){
                return m1.distance < m2.distance;
            }
            )->distance;
    match_3dpts_.clear();
    match_2dkp_index_.clear();

    // candidate 往外导

    for (auto m : matches){
        if (m.distance < max<float>(min_dis * match_ratio_, 30)){
            match_3dpts_.push_back(candidate[m.queryIdx]);
            match_2dkp_index_.push_back(m.trainIdx);
        }
    }
    cout << "good matches: " << match_3dpts_.size() << endl;
    cout << "match cost time: " << timer.elapsed() << endl;
}

// 3d to 2d
void VisualOdometry::poseEstimationPnP() {
    // 3D-2D
    vector<cv::Point3f> pts3d;
    vector<cv::Point2f> pts2d;

    for (int index: match_2dkp_index_){
        pts2d.push_back(keypoints_curr_[index].pt);
    }

    for (auto p: match_3dpts_){
        pts3d.push_back(p->getPositionCV());
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


    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,2>> Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverDense<Block::PoseMatrixType>();
    Block* solver_ptr = new Block( linearSolver );
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    // 添加点
    // new 的永远是指针
    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    // 这里我们只有一个pose
    pose->setId(0);
    pose->setEstimate(g2o::SE3Quat(
            T_c_r_estimated_.rotation_matrix(),
            T_c_r_estimated_.translation()
            ));
    optimizer.addVertex(pose);

    // 添加边
    for (int i = 0; i < num_inliers_; ++i) {
        // 首先，我需要把这一对点中第一个点的三维坐标放在point_中
        // 其次，我需要把第二个点的像素值放在测量中
        // 我需要把camera装在这个edge中。
        int index = inliers.at<int>(i, 0);
        myslam::EdgeProjectXYZ2UVPoseOnly* edge = new myslam::EdgeProjectXYZ2UVPoseOnly();
        edge->setId(i);
        edge->setVertex(0, pose);
        edge->camera_ = curr_->camera_.get();   // get 是智能指针中的方法
        edge->point_ = Vector3d(pts3d[index].x, pts3d[index].y, pts3d[index].z );
        edge->setMeasurement(Vector2d(pts2d[index].x, pts2d[index].y));
        edge->setInformation( Eigen::Matrix2d::Identity() );
        optimizer.addEdge(edge);

        match_3dpts_[index]->matched_times_++;
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);
    // 更新最后的T
    T_c_r_estimated_ = SE3(
            pose->estimate().rotation(),
            pose->estimate().translation()
            );
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

void VisualOdometry::addKeyFrame() {

    // 第一帧TODO 将所有的点都装到地图中
    if (map_->keyframes_.size() == 0){
        // 把所有的point都放在map中
        for (int i = 0; i < keypoints_curr_.size(); ++i) {
            Vector3d pos_world = curr_->camera_->pixel2world(
                    Vector2d( keypoints_curr_[i].pt.x,keypoints_curr_[i].pt.y),
                    curr_->T_c_w_,
                    curr_->findDepth(keypoints_curr_[i]));
            Vector3d n = pos_world - curr_->getCamCenter();
            n.normalize();
            // 智能指针的get()方法，能返回指针
            MapPoint::Ptr mapPoint = MapPoint::createMapPoint(pos_world,n,descriptors_curr_.row(i).clone(),curr_.get());
            map_->insertMapPoint(mapPoint);

        }
    }

    cout << "add a keyframe" << endl;
    map_->insertKeyFrame(curr_);
    // 插入关键帧的时候，同时更新参考帧位置， 参考帧用于初始值
    ref_ = curr_;
}

bool VisualOdometry::checkKeyFrame() {
    Sophus::Vector6d d = T_c_r_estimated_.log();
    Vector3d trans = d.head(3);
    Vector3d rot = d.tail(3);
    // cout << "位姿： trans: " << trans.transpose() << "  rot: "<< rot.transpose()<< endl;
    return rot.norm() > key_frame_min_rot || trans.norm() > key_frame_min_trans;
}

    void VisualOdometry::optimizeMap() {
        // 首先，如果不在curr_可视范围内，erase(无序map中的keypoint)

        // 其次， 匹配率 = 匹配的次数 / 看到的次数；如果匹配率太低，扔掉

        // 3. 太偏的mapPoint 扔掉 使用getViewAngle函数

        // 4. 依据mappoint的good属性，这部分主要是三角化，但是怎么

        // 太少的话，添加！

        // 太多的话，剔除，通过控制匹配率实现

    }

    void VisualOdometry::addMapPoints() {
        // 如果已经被匹配了的话，说明地图里面已经有该点，我们就不再添加
        vector<bool> matched(keypoints_curr_.size(), false);
        for (auto i: match_2dkp_index_){
            matched[i] = true;
        }

        for (int j = 0; j < keypoints_curr_.size(); ++j) {
            if (matched[j]){
                continue;
            }
            // 所有未被匹配的点中，我们需要get到深度
            double d;
            d = curr_->findDepth(keypoints_curr_[j]);
            if (d < 0){
                continue;
            }
            Vector3d pos_world = curr_->camera_->pixel2world(
                    Vector2d(keypoints_curr_[j].pt.x, keypoints_curr_[j].pt.y),
                    curr_->T_c_w_, d);          // this is your problem, always thinking something others;
            Vector3d n = pos_world - curr_->getCamCenter();

            MapPoint::Ptr mapPoint = MapPoint::createMapPoint(pos_world,n,descriptors_curr_.row(j).clone(),curr_.get());
            map_->insertMapPoint(mapPoint);
        }
    }

    double VisualOdometry::getViewAngle(Frame::Ptr frame, MapPoint::Ptr point) {

    // 这里是当前帧与参考关键帧对同一mappoint的视角夹角
    // TODO: 按照我写的，这里视角就是没啥变
    // 你想想怎么调整为，与关键帧的视角对比。
    // 代码大胆的写
        Vector3d n = point->pos_ - frame->getCamCenter();
        n.normalize();
        return acos(n.transpose() * point->norm_);
    }

}
