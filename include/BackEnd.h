#ifndef BACKEND_H
#define BACKEND_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <map>

#include "include/Common.h"
#include "include/Map.h"
#include "include/g2o_types.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace usrl_vo {

class Map;
class MapPoint;
class Frame;

class BackEnd {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    BackEnd(const std::string &setting_path, Map* map);

    // void SetMap(std::shared_ptr<Map> map);

    // void SetCameras(Camera::Ptr left, Camera::Ptr right);

    void UpdateMap();

    void Stop();

private:
    void BackEndLoop();

    void Optimize(std::unordered_map<unsigned long, Frame*> keyframes, 
                  std::unordered_map<unsigned long, MapPoint*> landmarks);

    Map* map_;
    std::thread backend_thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> backend_running_;

    // Camera::Ptr camera_left_ = nullptr;
    // Camera::Ptr camera_right_ = nullptr;

    cv::Mat K_;
    float bf_;

};

}

#endif  // BACKEND_H