#ifndef CONVERTER_H
#define CONVERTER_H

#include "include/Common.h"

#include<opencv2/core/core.hpp>
#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace usrl_vo {

class Converter {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);

    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);

    static cv::Mat toCvMat(const Eigen::Vector3d &m);
    
    static Eigen::Matrix<double,3,3> toMatrix33d(const cv::Mat &cvMat4);

    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);

    #if 0
    Vec3d WorldToCamera(const Vec3d &p_w, const SE3 &T_cw);

    Vec3d CameraToWorld(const Vec3d &p_c, const SE3 &T_cw);

    Vec2d CameraToPixel(const Vec3d &p_c, const Mat33d &K);

    Vec3d PixelToCamera(const Vec2d &p_uv, const double &depth, const Mat33d &K);

    Vec2d WorldToPixel(const Vec3d &p_w, const SE3 &T_cw, const Mat33d &K);

    Vec3d PixelToWorld(const Vec2d &p_uv, const SE3 &T_cw, const double &depth, const Mat33d &K);
    #endif
};

}

#endif