#include <ros/ros.h>
#include "tzj_slam/feature_extract.hpp"

using namespace cv;

int main() {

    cv::Mat frame, preframe;
    cv::VideoCapture cap(0);

    cap >> preframe;

    while ( true ) {
        cap >> frame ;
        findmatch_ORB( preframe, frame );

        preframe = frame;

        if (cv::waitKey(30) == 'q') {
            break;
        }
    }
    return 0;
}