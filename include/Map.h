#ifndef MAP_H
#define MAP_H

#include "include/Common.h"
#include "include/Frame.h"
#include "include/MapPoint.h"
#include "include/Feature.h"
#include "include/Converter.h"

namespace usrl_vo {

class Frame;
class MapPoint;
class Feature;

class Map {

// public:
//     typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
//     typedef std::unordered_map<unsigned long, Frame::Ptr> KeyframesType;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Map();

    void InsertKeyFrame(Frame* frame);

    void InsertMapPoint(MapPoint* map_point);

    std::unordered_map<unsigned long, MapPoint*> GetAllMapPoints();

    std::unordered_map<unsigned long, Frame*> GetAllKeyFrames();

    std::unordered_map<unsigned long, MapPoint*> GetActiveMapPoints();

    std::unordered_map<unsigned long, Frame*> GetActiveKeyFrames();

    long unsigned int ActiveKeyFramesInMap();

    void CleanMap();

    std::mutex map_update_mutex_;

private:
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    std::unordered_map<unsigned long, MapPoint*> landmarks_;
    std::unordered_map<unsigned long, MapPoint*> active_landmarks_;
    std::unordered_map<unsigned long, Frame*> keyframes_;
    std::unordered_map<unsigned long, Frame*> active_keyframes_;

    Frame* current_frame_;

    unsigned int num_active_keyframes_ = 7;
};

}

#endif