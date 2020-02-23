#ifndef VIEWER_H
#define VIEWER_H

#include <thread>
#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

#include "include/Common.h"
#include "include/Frame.h"
#include "include/Map.h"
#include "include/Feature.h"
#include "Converter.h"

namespace usrl_vo {

class Map;
class Frame;
class Converter;

class Viewer {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    Viewer(Map* map);

    void Close();

    void AddCurrentFrame(Frame* current_frame);

    void UpdateMap();

private:
    void Run();

    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);

    void DrawMapPoints();

    cv::Mat DrawFrame();

    Map* map_;
    Frame* current_frame_;
    
    std::thread viewer_thread_;
    bool viewer_running_ = true;

    std::unordered_map<unsigned long, Frame*> active_keyframes_;
    std::unordered_map<unsigned long, MapPoint*> active_landmarks_;
    bool map_updated_ = false;

    std::mutex viewer_data_mutex_;

    int num_tracked_, num_rackedVO_;
};

}

#endif