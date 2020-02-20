
#ifndef G2O_TYPES_H
#define G2O_TYPES_H

#include "include/Common.h"

#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_binary_edge.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/solver.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
// #include "Thirdparty/g2o/g2o/solvers/csparse/linear_solver_csparse.h"
// #include "Thirdparty/g2o/g2o/solvers/dense/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"

#include<Eigen/StdVector>

namespace usrl_vo {

#if 0
class VertexPose : public g2o::BaseVertex<6, SE3> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexPose() {}

    virtual bool read(std::istream& is) { return true; };

    virtual bool write(std::ostream& os) const { return true; };

    virtual void setToOriginImpl() {
        _estimate = SE3();
    }

    virtual void oplusImpl(const double *update) {
        Vec6d update_T;
        update_T << update[0], update[1], update[2], update[3], update[5], update[5];
        _estimate = SE3::exp(update_T) * _estimate;
    }
};

class VertexXYZ : public g2o::BaseVertex<3, Vec3d> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    VertexXYZ() {}

    virtual bool read(std::istream& is) { return true; };

    virtual bool write(std::ostream& os) const { return true; };

    virtual void setToOriginImpl() {
        _estimate.fill(0);
    }

    virtual void oplusImpl(const double *update) {
        Eigen::Map<const Vec3d> v(update);
        _estimate += v;
    }
};

class EdgeSE3ProjectXYZOnlyPose : public g2o::BaseUnaryEdge<2, Vec2d, VertexPose> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeSE3ProjectXYZOnlyPose(const Vec3d &pos, const Mat33d &K) : pos3d_(pos), K_(K) {}

    virtual bool read(std::istream &is) { return true; };

    virtual bool write(std::ostream &os) const { return true; };

    virtual void computeError() {
        const VertexPose *vi = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = vi->estimate();
        Vec3d pos_pixel = K_ * (T * pos3d_);
        pos_pixel /= pos_pixel[2];
        _error = _measurement - pos_pixel.head<2>();
    }

    virtual void linearizeOplus() {
        const VertexPose *vi = static_cast<VertexPose *>(_vertices[0]);
        SE3 T = vi->estimate();
        Vec3d pos_cam = T * pos3d_;

        double fx = K_(0, 0);
        double fy = K_(1, 1);
        double x = pos_cam[0];
        double y = pos_cam[1];
        double z_inv = 1.0 / (pos_cam[2] + 1e-18);
        double z_inv_2 = z_inv * z_inv;

        _jacobianOplusXi(0, 0) = -z_inv * fx;
        _jacobianOplusXi(0, 1) = 0;
        _jacobianOplusXi(0, 2) = x * z_inv_2 * fx;
        _jacobianOplusXi(0, 3) = x * y * z_inv_2 * fx;
        _jacobianOplusXi(0, 4) = -(1 + x * x * z_inv_2) * fx;
        _jacobianOplusXi(0, 5) = y * z_inv * fx;

        _jacobianOplusXi(1, 0) = 0;
        _jacobianOplusXi(1, 1) = -z_inv * fy;
        _jacobianOplusXi(1, 2) = y * z_inv_2 * fy;
        _jacobianOplusXi(1, 3) = (1 + y * y * z_inv_2) * fy;
        _jacobianOplusXi(1, 4) = -x * y * z_inv_2 * fy;
        _jacobianOplusXi(1, 5) = -x * z_inv * fy;
    }

private:
    Vec3d pos3d_;
    Mat33d K_;
};
#endif

class EdgeStereoProjectXYZ : public g2o::BaseBinaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap, g2o::VertexSBAPointXYZ> {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeStereoProjectXYZ() {}

    Eigen::Vector2d cam_project(const Eigen::Vector3d &trans_xyz, const float &bf) const {
        const float z_inv = 1.0f / (trans_xyz[2] + 1e-18);
        Eigen::Vector2d res;
        res[0] = trans_xyz[0] * z_inv * fx + cx - bf * z_inv;
        res[1] = trans_xyz[1] * z_inv * fy + cy;
        return res;
    }

    virtual void computeError() {
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        g2o::VertexSBAPointXYZ *vj = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[1]);
        Eigen::Vector2d obs(_measurement);
        _error = _measurement - cam_project(vi->estimate().map(vj->estimate()), bf);
        
    }

    virtual void linearizeOplus() {
        g2o::VertexSE3Expmap *vi = static_cast<g2o::VertexSE3Expmap *>(_vertices[0]);
        cv::Mat T =  Converter::toCvMat(vi->estimate());
        g2o::VertexSBAPointXYZ *vj = static_cast<g2o::VertexSBAPointXYZ *>(_vertices[1]);
        cv::Mat pos_w = Converter::toCvMat(vj->estimate());
        cv::Mat R_cw = T.rowRange(0,3).colRange(0,3);
        cv::Mat t_cw = T.rowRange(0,3).col(3);
        cv::Mat pos_cam = R_cw * pos_w + t_cw;

        double x = pos_cam.at<float>(0);
        double y = pos_cam.at<float>(1);
        double z_inv = 1.0 / (pos_cam.at<float>(2) + 1e-18);
        double z_inv_2 = z_inv * z_inv;

        Eigen::Matrix<double, 3, 3> R = Converter::toMatrix33d(T);

        _jacobianOplusXi(0, 0) = -fx * z_inv;
        _jacobianOplusXi(0, 1) = 0;
        _jacobianOplusXi(0, 2) = (x * fx - bf) * z_inv_2;
        _jacobianOplusXi(0, 3) = (x * y * fx - y * bf) * z_inv_2;
        _jacobianOplusXi(0, 4) = -fx - (x * x * fx - x * bf) * z_inv_2;
        _jacobianOplusXi(0, 5) = y * fx * z_inv;

        _jacobianOplusXi(1, 0) = 0;
        _jacobianOplusXi(1, 1) = -fy * z_inv;
        _jacobianOplusXi(1, 2) = y * fy * z_inv_2;
        _jacobianOplusXi(1, 3) = (1 + y * y * z_inv_2) * fy;
        _jacobianOplusXi(1, 4) = -x * y * fy * z_inv_2;
        _jacobianOplusXi(1, 5) = -x * fy * z_inv;

        _jacobianOplusXj(0, 0) = -fx * z_inv * R(0, 0) + (x * z_inv_2 * fx - z_inv_2 * bf) * R(2, 0);
        _jacobianOplusXj(0, 1) = -fx * z_inv * R(0, 1) + (x * z_inv_2 * fx - z_inv_2 * bf) * R(2, 1);
        _jacobianOplusXj(0, 2) = -fx * z_inv * R(0, 2) + (x * z_inv_2 * fx - z_inv_2 * bf) * R(2, 2);

        _jacobianOplusXj(1, 0) = -fy * z_inv * R(1, 0) + fy * y * z_inv_2 * R(2, 0);
        _jacobianOplusXj(1, 1) = -fy * z_inv * R(1, 1) + fy * y * z_inv_2 * R(2, 1);
        _jacobianOplusXj(1, 2) = -fy * z_inv * R(1, 2) + fy * y * z_inv_2 * R(2, 2);
    }

    virtual bool read(std::istream &is) { return true; };

    virtual bool write(std::ostream &os) const { return true; };

    double fx, fy, cx, cy, bf;
};

}

#endif