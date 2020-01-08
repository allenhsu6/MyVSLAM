add_library(myslam SHARED
        Camera.cpp
        Config.cpp
        Map.cpp
        MapPoint.cpp
        Frame.cpp
        visual_odometry.cpp
        g2o_types.cpp)

target_link_libraries(myslam ${THIRD_PARTY_LIBS})