#ifndef FRONTEND_H
#define FRONTEND_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <vector>

#include "include/Common.h"
#include "include/Frame.h"
#include "include/Feature.h"
#include "include/MapPoint.h"
#include "include/Map.h"
#include "include/Converter.h"
#include "include/Viewer.h"
#include "include/g2o_types.h"
#include "include/BackEnd.h"

namespace usrl_vo {

class BackEnd;
class Viewer;
class Frame;
class Map;
class MapPoint;
class Feature;
    
class FrontEnd {

public:

    enum eFrontEndStatus {
        NO_IMAGES,
        NOT_INITIALIZED,
        TRACKING_GOOD,
        TRACKING_BAD,
        LOST
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    FrontEnd(const std::string &setting_path, Map* map, BackEnd* backend);

    void Track();

    // void SetMap(Map* map);

    // void SetBackEnd(BackEnd* backend);

    void SetViewer(Viewer* viewer);

    // inline FrontEndStatus GetStatus() {
    //     return status_;
    // }

    // void SetCameras(Camera::Ptr left, Camera::Ptr right);

    cv::Mat ProcessImageStereo(const cv::Mat &im_rect_left, const cv::Mat &im_rect_right, const double &time_stamp);

private:
    void StereoInit();

    int DetectFeatures();

    int FindFeaturesInRight();

    bool BuildInitMap();

    bool TrackLoop();

    int TrackLastFrame();

    int EstimateCurrentPose();

    bool CreateNewKeyFrame();

    void SetObservationsForKeyFrame();

    int TriangulateNewPoints();

    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &p_3d);

    // Vec3d WorldToCamera(const Vec3d &p_w, const SE3 &T_cw, const SE3 &ext);

    // Vec2d CameraToPixel(const Vec3d &p_c, const Mat33d &K);

    cv::Point2f WorldToPixelinLeft(const cv::Mat &p_w, const cv::Mat &T_cw, const cv::Mat &K);

    cv::Point2f WorldToPixelinRight(const cv::Mat &p_w, const cv::Mat &T_cw, const cv::Mat &K, const float &bf);

    bool Reset();

    eFrontEndStatus status_;

    Map* map_;
    BackEnd* backend_ ;
    Viewer* viewer_;

    Frame* current_frame_;
    // Frame* new_frame_;
    Frame* last_frame_;
    // Camera::Ptr camera_left_ = nullptr;
    // Camera::Ptr camera_right_ = nullptr;

    std::list<cv::Mat> relative_poses_;

    int tracking_inliers_ = 0;

    int num_features_ = 200;
    int num_features_init_ = 100;
    int num_features_tracking_good_ = 30;
    int num_features_tracking_bad_ = 10;
    int num_features_needed_for_keyframe_ = 60;

    cv::Ptr<cv::GFTTDetector> gftt_;

    cv::Mat K_;
    cv::Mat R_rl_;
    cv::Mat t_rl_;
    float bf_;
};

}

#endif  // FRONTEND_H