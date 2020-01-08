//
// Created by xu on 2019/12/12.
//

#include "../include/Map.h"

namespace myslam
{

void myslam::Map::insertKeyFrame(Frame::Ptr frame) {
    cout << "key frame size = " << keyframes_.size() << endl;
    // 给map添加
    if (keyframes_.find(frame->id_) == keyframes_.end()){
        // 添加
        keyframes_.insert(make_pair(frame->id_, frame));
    }else{
        // 更新
        keyframes_[frame->id_] = frame;
    }
}

// 地图点的添加与更新
void myslam::Map::insertMapPoint(MapPoint::Ptr map_point) {
    cout << "map_point size = " << map_points_.size() << endl;
    if (map_points_.find((map_point->id_)) == map_points_.end()){
       map_points_.insert(make_pair(map_point->id_, map_point));
    } else{
        map_points_[map_point->id_] = map_point;
    }
}

}

