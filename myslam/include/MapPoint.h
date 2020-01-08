//
// Created by xu on 2019/12/12.
// 有一种在创建一个生命的感觉，赋予它一些属性与功能
// 这个类的设计，有点东西

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include <common_include.h>

namespace myslam
{
class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;  // ID
    static unsigned long factory_id_;   // factory id
    Vector3d pos_;      // Position in world
    Vector3d norm_;     // Normal of viewing direction  相机中心指向mappoint，可以表示距离，朝向等特征
    bool good;          // 是否保留  判断是否保留的时候，可以通过观测次数，inliner次数判断。
    Mat descriptor_;    // 用于与像素坐标匹配

    //  使用list 插入删除效率比较高 查找比较慢
    list<Frame*> observed_frames_;  // 可以观测到该点的frame

    int visible_time_; // being observed in current frame
    int matched_times_; // being an inliner in pose estimation

    MapPoint();
    MapPoint(
            long id,
            Vector3d position,
            Vector3d norm,
            Frame* frame= nullptr,
            const Mat& descriptor=Mat());

    inline cv::Point3f getPositionCV() const {
        return cv::Point3f(pos_[0], pos_[1], pos_[2]);
    }
    static MapPoint::Ptr createMapPoint(
            const Vector3d& pos_world,
            const Vector3d& norm_,
            const Mat& descriptor,
            Frame* frame
            );
};
}
#endif //MYSLAM_MAPPOINT_H
