#ifndef BACKEND_H
#define BACKEND_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Common.h"

namespace usrl_vo {

class Map;

class BackEnd {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<BackEnd> Ptr;

public:
    BackEnd();

    void SetMap(Map::Ptr map) {
        map_ = map
    };

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

    void UpdateMap();

    void Stop();

private:
    void BackEndLoop();

    void Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks);

    std::shared_ptr<Map> map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    Camera::Ptr cam_left_ = nullptr;
    Camera::Ptr cam_right_ = nullptr;

};

}

#endif  // BACKEND_H