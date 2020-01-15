#include "Camera.h"

namespace usrl_vo {

Camera::Camera() {
}

Camera::Camera(Mat33d &K)
      : K_(K.clone()) {
    
    fx_ = K.at<float>(0,0);
    fy_ = K.at<float>(1,1);
    cx_ = K.at<float>(0,2);
    cy_ = K.at<float>(1,2); 
}

Vec3 Camera::WorldToCamera(const Vec3 &p_w, const SE3 &T_cw) {
    return T_cw * p_w;
}

Vec3 Camera::CameraToWorld(const Vec3 &p_c, const SE3 &T_cw) {
    return T_cw.inverse() * p_c;
}

Vec2 Camera::CameraToPixel(const Vec3 &p_c) {
    return Vec2(
        fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
        fy_ * p_c(1, 0) / p_c(2, 0) + cy_
    );
}

Vec3 Camera::PixelToCamera(const Vec2 &p_uv, const double &depth) {
    return Vec3(
        (p_uv(0, 0) - cx_) * depth / fx_,
        (p_uv(1, 0) - cy_) * depth / fy_,
        depth)
    );
}

Vec2 Camera::WorldToPixel(const Vec3 &p_w, const SE3 &T_cw) {
    return camera2pixel(world2camera(p_w, T_cw));
}

Vec3 Camera::PixelToWorld(const Vec2 &p_uv, const SE3 &T_cw, const double &depth) {
    return camera2world(pixel2camera(p_uv, depth), T_cw);
}

}
