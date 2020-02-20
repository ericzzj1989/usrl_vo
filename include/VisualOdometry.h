#ifndef VISUALODOMETRY_H
#define VISUALODOMETRY_H

#include<string>
#include<thread>
#include<iomanip>
#include<opencv2/core/core.hpp>

#include "include/Common.h"
#include "include/BackEnd.h"
#include "include/FrontEnd.h"
#include "include/Viewer.h"

namespace usrl_vo {

class FrontEnd;
class BackEnd;
class Map;
class Viewer;

class VisualOdometry {

public:
    enum SensorType {
        MONOCULAR,
        STEREO,
        RGBD
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VisualOdometry(const std::string &setting_file, const SensorType sensor, const bool use_viewer = true);

    cv::Mat RunStereo(const cv::Mat &im_left, const cv::Mat &im_right, const double &time_stamp);

    void Shutdown();

private:
    
    FrontEnd* frontend_;
    
    BackEnd* backend_;
    
    Map* map_;
    
    Viewer* viewer_;

    std::thread* viewer_thread_;
    std::thread* backend_thread_;

    SensorType sensor_;
};

}

#endif