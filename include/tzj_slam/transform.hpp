#include <Eigen/Core>
#include <Eigen/Geometry>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for Sophus
#include <sophus/se3.h>
#include <sophus/so3.h>
using Sophus::SO3;
using Sophus::SE3;

// for cv
#include <opencv2/core/core.hpp>
using cv::Mat;

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>


Vector3d world2camera ( const Vector3d& p_w, const SE3& T_c_w );

Vector3d camera2world ( const Vector3d& p_c, const SE3& T_c_w );

Vector2d camera2pixel ( const Vector3d& p_c, double fx_, double cx_, double fy_, double cy_);

Vector3d pixel2camera ( const Vector2d& p_p, double depth, double fx_, double cx_, double fy_, double cy_);

Vector2d world2pixel ( const Vector3d& p_w, const SE3& T_c_w );

Vector3d pixel2world ( const Vector2d& p_p, const SE3& T_c_w, double depth );
