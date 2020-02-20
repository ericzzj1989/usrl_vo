#include "include/Converter.h"

namespace usrl_vo {

g2o::SE3Quat Converter::toSE3Quat(const cv::Mat &cvT)
{
    Eigen::Matrix<double,3,3> R;
    R << cvT.at<float>(0,0), cvT.at<float>(0,1), cvT.at<float>(0,2),
         cvT.at<float>(1,0), cvT.at<float>(1,1), cvT.at<float>(1,2),
         cvT.at<float>(2,0), cvT.at<float>(2,1), cvT.at<float>(2,2);

    Eigen::Matrix<double,3,1> t(cvT.at<float>(0,3), cvT.at<float>(1,3), cvT.at<float>(2,3));

    return g2o::SE3Quat(R,t);
}

cv::Mat Converter::toCvMat(const g2o::SE3Quat &SE3)
{
    Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();
    return toCvMat(eigMat);
}

cv::Mat Converter::toCvMat(const Eigen::Matrix<double,4,4> &m)
{
    cv::Mat cvMat(4, 4, CV_32F);
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
            cvMat.at<float>(i, j) = m(i, j);
    return cvMat.clone();
}

Eigen::Matrix<double,3,1> Converter::toVector3d(const cv::Mat &cvVector)
{
    Eigen::Matrix<double,3,1> v;
    v << cvVector.at<float>(0), cvVector.at<float>(1), cvVector.at<float>(2);

    return v;
}

// cv::Mat Converter::toCvMat(const Mat33d &m) {
//     cv::Mat cvMat(3,3,CV_32F);
//     for(int i=0;i<3;i++)
//         for(int j=0; j<3; j++)
//             cvMat.at<float>(i,j)=m(i,j);

//     return cvMat.clone();
// }

// cv::Mat Converter::toCvMat(const Eigen::Matrix<double,3,1> &m)
// {
//     cv::Mat cvMat(3,1,CV_32F);
//     for(int i=0;i<3;i++)
//             cvMat.at<float>(i)=m(i);

//     return cvMat.clone();
// }

cv::Mat Converter::toCvMat(const Eigen::Vector3d &m)
{
    cv::Mat cvMat(3,1,CV_32F);
    for(int i=0;i<3;i++)
            cvMat.at<float>(i)=m(i);

    return cvMat.clone();
}

Eigen::Matrix<double,3,3> Converter::toMatrix33d(const cv::Mat &cvMat4)
{
    Eigen::Matrix<double,3,3> M;

    M << cvMat4.at<float>(0,0), cvMat4.at<float>(0,1), cvMat4.at<float>(0,2),
         cvMat4.at<float>(1,0), cvMat4.at<float>(1,1), cvMat4.at<float>(1,2),
         cvMat4.at<float>(2,0), cvMat4.at<float>(2,1), cvMat4.at<float>(2,2);

    return M;
}

#if 0
Vec3d Converter::WorldToCamera(const Vec3d &p_w, const SE3 &T_cw) {
    return T_cw * p_w;
}

Vec3d Converter::CameraToWorld(const Vec3d &p_c, const SE3 &T_cw) {
    return T_cw.inverse() * p_c;
}

Vec2d Converter::CameraToPixel(const Vec3d &p_c, const Mat33d &K) {
    return Vec2d(
        K(0, 0) * p_c(0, 0) / p_c(2, 0) + K(0, 2),
        K(1, 1) * p_c(1, 0) / p_c(2, 0) + K(1, 2)
    );
}

Vec3d Converter::PixelToCamera(const Vec2d &p_uv, const double &depth, const Mat33d &K) {
    return Vec3d(
        (p_uv(0, 0) - K(0, 2)) * depth / K(0, 0),
        (p_uv(1, 0) - K(1, 2)) * depth / K(1, 1),
        depth);
}

Vec2d Converter::WorldToPixel(const Vec3d &p_w, const SE3 &T_cw, const Mat33d &K) {
    return CameraToPixel(WorldToCamera(p_w, T_cw), K);
}

Vec3d Converter::PixelToWorld(const Vec2d &p_uv, const SE3 &T_cw, const double &depth, const Mat33d &K) {
    return CameraToWorld(PixelToCamera(p_uv, depth, K), T_cw);
}
#endif

}