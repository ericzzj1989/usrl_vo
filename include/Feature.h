#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/features2d.hpp>

#include "include/Common.h"
#include "include/Frame.h"

namespace usrl_vo {

class Frame;
class MapPoint;

class Feature {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Feature> Ptr;

    Frame* frame_;
    cv::KeyPoint position_;
    MapPoint* map_point_;
    bool is_outlier_;
    bool is_on_left_image_;

public:
    Feature(Frame* frame, const cv::KeyPoint &kp);
};

}

#endif