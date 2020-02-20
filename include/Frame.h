#ifndef FRAME_H
#define FRAME_H

#include "include/Common.h"
#include "include/Camera.h"
#include "include/Feature.h"

namespace usrl_vo {

class Feature;
class MapPoint;
    
class Frame {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static long unsigned int num_next_frame_id;
    long unsigned int id_;

    static long unsigned int num_next_keyframe_id;
    long unsigned int keyframe_id_;

    bool is_keyframe_;

    cv::Mat left_img_;
    cv::Mat right_img_;

    double time_stamp_;

    cv::Mat pose_;

    std::vector<Feature*> features_left_;
    std::vector<Feature*> features_right_;

public:
    Frame(const Frame &frame);
    
    Frame(const cv::Mat &left, const cv::Mat &right, const double &time_stampt);

    void SetPose(const cv::Mat &pose);

    //Tcw
    cv::Mat GetPose();

    //Twc
    cv::Mat GetPoseInverse();

    void SetKeyFrame();

    // static Frame* CreateFrame();

protected:
    cv::Mat pose_inv_;
    cv::Mat Rcw_;
    cv::Mat tcw_;
    cv::Mat Rwc_;
    cv::Mat twc_; //==mtwc

    std::mutex pose_mutex_;

};

}

#endif