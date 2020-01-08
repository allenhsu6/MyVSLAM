//
// Created by xu on 2019/12/12.
//

#include "../include/MapPoint.h"

namespace myslam {

// 两个构造函数
// 想清楚， 每个关键点都有对应的frame关联，观测次数
MapPoint::MapPoint()
        : id_(-1), pos_(Vector3d(0, 0, 0)), norm_(Vector3d(0, 0, 0)), good(true), visible_time_(0), matched_times_(0) {
}

MapPoint::MapPoint(long id, Vector3d position, Vector3d norm, Frame *frame, const Mat &descriptor)
    :good(true), id_(id), pos_(position), norm_(norm), descriptor_(descriptor), visible_time_(1), matched_times_(1){
    observed_frames_.push_back(frame);
}


MapPoint::Ptr
MapPoint::createMapPoint(const Vector3d &pos_world, const Vector3d &norm_, const Mat &descriptor, Frame *frame) {


return  MapPoint::Ptr(new MapPoint(factory_id_++, pos_world, norm_, frame, descriptor)) ;
}

unsigned long MapPoint::factory_id_ = 0;
}