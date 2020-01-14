#ifndef FRONTEND_H
#define FRONTEND_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

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
    
class Frontend {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<FrontEnd> Ptr;

public:
    Frontend();

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

    FrontEndStatus GetStatus() const {
        return status_;
    }

    void SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

private:
    bool StereoInit();

};

}