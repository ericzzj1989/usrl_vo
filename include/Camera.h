#ifndef CAMERA_H
#define CAMERA_H

#include "Common.h"

namespace usrl_vo {

class Camera {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Camera> Ptr;

public:
    Camera();

    Camera(Mat33d &K);

    Vec3d WorldToCamera(const Vec3d &p_w, const SE3 &T_cw);

    Vec3d CameraToWorld(const Vec3d &p_c, const SE3 &T_cw);

    Vec2d CameraToPixel(const Vec3d &p_c);

    Vec3d PixelToCamera(const Vec2d &p_uv, const double &depth);

    Vec2d WorldToPixel(const Vec3d &p_w, const SE3 &T_cw);

    Vec3d PixelToWorld(const Vec2d &p_uv, const SE3 &T_cw, const double &depth);

public:
    Mat33d K_;
    static float fx_;
    static float fy_;
    static float cx_;
    static float cy_;

};

} 

#endif // CAMERA_H