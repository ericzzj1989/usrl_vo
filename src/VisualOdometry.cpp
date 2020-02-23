#include "include/VisualOdometry.h"

namespace usrl_vo {

VisualOdometry::VisualOdometry(const std::string &settings_file, const SensorType sensor, const bool use_viewer) 
                              : sensor_(sensor)
{
    std::cout << "Input sensor was set to: ";

    if(SensorType::MONOCULAR == sensor_) 
        std::cout << "Monocular" << std::endl;
    else if(SensorType::STEREO == sensor_)
        std::cout << "Stereo" << std::endl;
    else if(SensorType::RGBD == sensor_)
        std::cout << "RGB-D" << std::endl;

    cv::FileStorage f_settings(settings_file.c_str(), cv::FileStorage::READ);
    if(!f_settings.isOpened()) {
        std::cerr << "Failed to open settings file at: " << settings_file << std::endl;
        exit(-1);
    }

    map_ = new Map();

    backend_ = new BackEnd(settings_file, map_);
    // backend_thread_ = new thread(&usrl_vo::BackEnd::BackEndLoop);

    frontend_ = new FrontEnd(settings_file, map_, backend_);

    if(use_viewer)
    {
        viewer_ = new Viewer(map_);
        // viewer_thread_ = new std::thread(&Viewer::Run, viewer_));
        frontend_->SetViewer(viewer_);
    }
}

cv::Mat VisualOdometry::RunStereo(const cv::Mat &im_left, const cv::Mat &im_right, const double &time_stamp)
{
    if(STEREO != sensor_)
    {
        std::cerr << "Error: you called RunStereo but input sensor was not set to STEREO." << std::endl;
        exit(-1);
    }

    cv::Mat T_cw = frontend_->ProcessImageStereo(im_left, im_right, time_stamp);

    return T_cw;
}

void VisualOdometry::Shutdown()
{
    backend_->Stop();

    if(viewer_)
    {
        viewer_->Close();
    }

    std::cout << std::endl << "VO exit" << std::endl;
}

}