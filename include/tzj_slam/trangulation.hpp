#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;

void triangulation ( 
    const std::vector< cv::KeyPoint >& keypoint_1, 
    const std::vector< cv::KeyPoint >& keypoint_2, 
    const std::vector< cv::DMatch >& matches,
    const cv::Mat& R, const cv::Mat& t, 
    std::vector< cv::Point3f >& points );
