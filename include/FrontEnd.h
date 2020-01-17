#ifndef FRONTEND_H
#define FRONTEND_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Common.h"
#include "Frame.h"
#include "Feature.h"
#include "MapPoint.h"
#include "Map.h"

namespace usrl_vo {

enum class FrontEndStatus {
    INITING,
    TRACKING_GOOD,
    TRACKING_BAD,
    LOST
};
    
class FrontEnd {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<FrontEnd> Ptr;

public:
    FrontEnd();

    bool AddFrame(Frame::Ptr frame);

    void SetMap(Map::Ptr map) {
        map_ = map
    };

    void SetBackEnd(std::shared_ptr<Backend> backend) {
        backend_ = backend;
    }

    void SetViewer(std::shared_ptr<Viewer> viewer) {
        viewer_ = viewer;
    }

    inline FrontEndStatus GetStatus() {
        return status_;
    }

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    bool StereoInit();

    int DetectFeatures();

    int FindFeaturesInRight();

    bool BuildInitMap();

    bool Track();

    int TrackLastFrame();

    int EstimateCurrentPose();

    bool AddKeyFrame();

    void SetObservationsForKeyFrame();

    int TriangukateNewPoints();

    bool Reset();

    FrontEndStatus status_ = FrontEndStatus::INITING;

    Frame::Ptr current_frame_ = nullptr;
    Frame::Ptr last_frame_ = nullptr;
    Camera::Ptr camera_left_ = nullptr;
    Camera::Ptr camera_right_ = nullptr;

    Map::Ptr map_ = nullptr;
    std::shared_ptr<BackEnd> backend_ = nullptr;
    std::shared_ptr<Viewer> viewer_ = nullptr;

    SE3 relative_motion_;

    int tracking_inliers_ = 0;

    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_good_ = 50;
    int num_features_tracking_bad_ = 20;
    int num_features_needed_for_keyframe_ = 80;

    cv::Ptr<cv::GFTTDetector> gftt_;
};

}

#endif  // FRONTEND_H