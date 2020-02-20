#include "include/Camera.h"

namespace usrl_vo {

// Camera::Camera(Mat33d &K)
//       : K_(K) {
    
//     fx_ = K.at<float>(0,0);
//     fy_ = K.at<float>(1,1);
//     cx_ = K.at<float>(0,2);
//     cy_ = K.at<float>(1,2); 
// }

// Vec3d Camera::WorldToCamera(const Vec3d &p_w, const SE3 &T_cw) {
//     return T_cw * p_w;
// }

// Vec3d Camera::CameraToWorld(const Vec3d &p_c, const SE3 &T_cw) {
//     return T_cw.inverse() * p_c;
// }

// Vec2d Camera::CameraToPixel(const Vec3d &p_c) {
//     return Vec2d(
//         fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
//         fy_ * p_c(1, 0) / p_c(2, 0) + cy_
//     );
// }

// Vec3d Camera::PixelToCamera(const Vec2d &p_uv, const double &depth) {
//     return Vec3d(
//         (p_uv(0, 0) - cx_) * depth / fx_,
//         (p_uv(1, 0) - cy_) * depth / fy_,
//         depth);
// }

// Vec2d Camera::WorldToPixel(const Vec3d &p_w, const SE3 &T_cw) {
//     return CameraToPixel(WorldToCamera(p_w, T_cw));
// }

// Vec3d Camera::PixelToWorld(const Vec2d &p_uv, const SE3 &T_cw, const double &depth) {
//     return CameraToWorld(PixelToCamera(p_uv, depth), T_cw);
// }

}
