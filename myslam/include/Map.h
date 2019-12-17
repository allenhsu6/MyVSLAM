//
// Created by xu on 2019/12/12.
//

#ifndef MYSLAM_MAP_H
#define MYSLAM_MAP_H

#include <common_include.h>
#include <MapPoint.h>

namespace myslam
{
class Map
{
public:
    typedef shared_ptr<Map> Ptr;
    unordered_map<unsigned long, MapPoint::Ptr> map_points_;    // all_landmark
    unordered_map<unsigned long, Frame::Ptr> keyframes_;        // all key_frames

    Map() {}

    void insertKeyFrame(Frame::Ptr frame);
    void insertMapPoint(MapPoint::Ptr map_point);

};
}

#endif //MYSLAM_MAP_H
