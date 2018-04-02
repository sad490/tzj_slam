#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

void pose_estimation_2d2d ( std::vector<cv::KeyPoint> keypoints_1,
                            std::vector<cv::KeyPoint> keypoints_2,
                            std::vector<cv::DMatch > matches,
                            cv::Mat& R, cv::Mat& t );

void bundleAdjustment (
    const std::vector< cv::Point3f > points_3d,
    const std::vector< cv::Point2f > points_2d,
    const cv::Mat& K,
    cv::Mat& R, cv::Mat& t );
