#pragma once
#ifndef MAP_H
#define MAP_H

namespace usrl_vo {

struct Map {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Map> Ptr;
    typedef std::unordered_map<unsigned long, MapPoint::Ptr> LandmarksType;
    typedef std::unordered_map<unsigned long, Frame:Ptr> KeyframesType;

public:
    Map() {}

    void InsertKeyFrame(Frame::Ptr frame);

    void InsertMapPoint(MapPoint::Ptr map_point);

    LandmarksType GeetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

    KeyframesType GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

    LandmarksType GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

    KeyframesType GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

    void CleanMap();

private:
    void RemoveOldKeyframe();

    std::mutex data_mutex_;
    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyframesType keyframes_;
    KeyframesType active_keyframes_;

    Frame::Ptr current_frame_ = nullptr;

    int num_active_keyframes_ = 7;

};

}

#endif