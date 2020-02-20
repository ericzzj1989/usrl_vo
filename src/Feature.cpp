#include "include/Feature.h"

namespace usrl_vo {

Feature::Feature(Frame* frame, const cv::KeyPoint &kp):
    frame_(frame), position_(kp) 
{
    map_point_ = static_cast<MapPoint*>(NULL);

    is_outlier_ = false;

    is_on_left_image_ = true;
}

}