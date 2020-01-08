#pragma once

#ifndef FEATURE_H
#define FEATURE_H

namespace usrl_vo {

struct Feature {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    std::weak_ptr<Frame> frame_;
    cv::KeyPoint position_;
    std::weak_ptr<MapPoint> map_point_;
    bool is_outlier_ = false;
    bool is_on_left_image_ = true;

public: 
    Feature() {}

    Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
    : frame_(frame), position_(kp) {}

};

}

#endif