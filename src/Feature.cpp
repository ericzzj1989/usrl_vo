#include "Feature.h"

namespace usrl_vo {

    Feature::Feature() {

    }

    Feature::Feature(std::shared_ptr<Frame> frame, const cv::KeyPoint &kp)
    : frame_(frame), position_(kp) {
        
    }
}