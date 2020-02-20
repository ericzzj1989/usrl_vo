#ifndef COMMON_H
#define COMMON_H

#include <atomic>
#include <algorithm>
#include <condition_variable>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <typeinfo>
#include <unordered_map>
#include <vector>
#include <unistd.h>
#include <iterator>

#include<opencv2/core/core.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

// #include <sophus/se3.hpp>
// #include <sophus/so3.hpp>

namespace usrl_vo {

typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::Dynamic> MatXXd;
typedef Eigen::Matrix<double, 10, 10> Mat1010d;
typedef Eigen::Matrix<double, 13, 13> Mat1313d;
typedef Eigen::Matrix<double, 8, 10> Mat810d;
typedef Eigen::Matrix<double, 4, 4> Mat44d;
typedef Eigen::Matrix<double, 3, 4> Mat34d;
typedef Eigen::Matrix<double, 3, 3> Mat33d;
typedef Eigen::Matrix<double, 2, 2> Mat22d;

typedef Eigen::Matrix<double, Eigen::Dynamic, 1> VecXd;
typedef Eigen::Matrix<double, 6, 1> Vec6d;
typedef Eigen::Matrix<double, 3, 1> Vec3d;
typedef Eigen::Matrix<double, 2, 1> Vec2d;

// typedef Sophus::SE3d SE3;
// typedef Sophus::SO3d SO3;

}

#endif // COMMON_H