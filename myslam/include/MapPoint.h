//
// Created by xu on 2019/12/12.
//

#ifndef MYSLAM_MAPPOINT_H
#define MYSLAM_MAPPOINT_H

#include <common_include.h>
#include <Frame.h>
namespace myslam
{
class Frame;
class MapPoint
{
public:
    typedef shared_ptr<MapPoint> Ptr;
    unsigned long id_;  // ID
    Vector3d pos_;      // Position in world
    Vector3d norm_;     // Normal of viewing direction
    Mat descriptor_;    // Descriptor for matching
    int observed_time_; // being observed by feature
    int correct_times_; // being an inliner in pose estimation

    MapPoint();
    MapPoint(long id, Vector3d position, Vector3d norm);

    static MapPoint::Ptr createMapPoint();
};
}
#endif //MYSLAM_MAPPOINT_H
