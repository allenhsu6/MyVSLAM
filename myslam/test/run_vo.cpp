//
// Created by xu on 2019/12/12.
//

// test the visual odometry

#include <fstream>
#include <boost/timer.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/viz.hpp>

#include "Config.h"
#include "visual_odometry.h"



int main(int argc, char** argv){
    if (argc != 2){
        cout << "usage: run_vo parameter_file" << endl;
        return 1;
    }
    myslam::Config::setParameterFile(argv[1]);
    myslam::VisualOdometry::Ptr vo(new myslam::VisualOdometry);

    string dataset_dir = myslam::Config::get<string>("dataset_dir");
    cout << "dataset: " << dataset_dir << endl;
    ifstream fin(dataset_dir + "/associate.txt");
    if (!fin){
        cout << "Please ensure the associate.txt existed" << endl;
        return 1;
    }

    vector<string> rgb_files, depth_files;
    vector<double> rgb_times, depth_times;

    while(!fin.eof()){
        string rgb_file, depth_file, rgb_time, depth_time;
        fin >> rgb_time >> rgb_file >> depth_time >> depth_file;
        rgb_files.push_back(dataset_dir+"/"+rgb_file);
        depth_files.push_back(dataset_dir+"/"+depth_file);
        rgb_times.push_back(atof(rgb_time.c_str()));
        depth_times.push_back(atof(depth_time.c_str()));

        if (!fin.good()){
            break;
        }
    }

    // 智能指针的使用 是定义好类型之后，直接装填。// camera 参数有问题
    myslam::Camera::Ptr camera (new myslam::Camera);

    // 开始画图
    cv::viz::Viz3d vis("visual odometry");
    cv::viz::WCoordinateSystem world_coor(1);
    cv::viz::WCoordinateSystem camera_coor(0.5);
    cv::Point3d cam_pos( 0, -1.0, -1.0 ), cam_focal_point(0,0,0), cam_y_dir(0,1,0);
    cv::Affine3d cam_pose = cv::viz::makeCameraPose( cam_pos, cam_focal_point, cam_y_dir );

    vis.setViewerPose(cam_pose);

    // 设置坐标轴的参数
    world_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 1.0);
    camera_coor.setRenderingProperty(cv::viz::LINE_WIDTH, 0.5);

    vis.showWidget("World", world_coor);
    vis.showWidget("Camera", camera_coor);

    cout << "read total: " << rgb_files.size() << endl;
    for (int i = 0; i < rgb_files.size(); ++i) {

        // 把第i个中的时间 mat取出来，
        Mat color = cv::imread(rgb_files[i]);
        Mat depth = cv::imread(depth_files[i], -1);

        if (color.data == nullptr || depth.data == nullptr){
            break;
        }
        // 先设置frame, 静态函数
        myslam::Frame::Ptr pFrame = myslam::Frame::createFrame();
        pFrame->camera_ = camera;
        pFrame->depth_ = depth;
        pFrame->color_ = color;
        pFrame->time_stamp_ = rgb_times[i];

        boost::timer timer;
        vo->addFrame(pFrame);
        cout << "VO cost: " << timer.elapsed() << " in this frame" << endl;

        //  判断是否lost，常规操作是，遇到lost启动重定位
        if (vo->state_ == myslam::VisualOdometry::LOST){
            break;
        }

        SE3 Tcw = pFrame->T_c_w_.inverse(); // 相机相对于world的位置

        // show the map and the camera pose
        cv::Affine3d M(
                // 旋转矩阵
                cv::Affine3d::Mat3(
                        Tcw.rotation_matrix()(0,0), Tcw.rotation_matrix()(0,1), Tcw.rotation_matrix()(0,2),
                        Tcw.rotation_matrix()(1,0), Tcw.rotation_matrix()(1,1), Tcw.rotation_matrix()(1,2),
                        Tcw.rotation_matrix()(2,0), Tcw.rotation_matrix()(2,1), Tcw.rotation_matrix()(2,2)
                ),
                // 平移向量
                cv::Affine3d::Vec3(
                        Tcw.translation()(0,0), Tcw.translation()(1,0), Tcw.translation()(2,0)
                )
        );
        cv::imshow("image", color );
        cv::waitKey(1);
        vis.setWidgetPose( "Camera", M);
        vis.spinOnce(1, false);
    }

    cout << "关键帧数量： " << vo->map_->keyframes_.size() << endl;
    return 0;
}