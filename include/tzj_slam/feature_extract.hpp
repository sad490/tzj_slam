
#include <opencv2/opencv.hpp>
#include <vector>

std::vector<cv::DMatch> findmatch_ORB( cv::Mat& src1, cv::Mat& src2, std::vector<cv::KeyPoint>& kp1,  std::vector<cv::KeyPoint>& kp2 );
std::vector<cv::KeyPoint> get_keypoints( const cv::Mat& frame );