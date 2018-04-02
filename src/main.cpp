#include <ros/ros.h>
#include <cmath>
#include "tzj_slam/feature_extract.hpp"
#include "tzj_slam/pose_estimation.hpp"
#include "tzj_slam/trangulation.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>

using namespace cv;
using namespace std;

int main( int argc, char **argv ) {

    ros::init(argc,  argv, "pcl_creater");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("pcl_output", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Point> ("pose", 1);
    pcl::PointCloud<pcl::PointXYZ> cloud;
    sensor_msgs::PointCloud2 output;

    cv::Mat frame, preframe;
    cv::VideoCapture cap(0);
    cap >> preframe;

    vector<DMatch> matches ;
    vector<KeyPoint> kp1, kp2;
    Mat R, t;
    vector<Point3f> points; 
    vector<Point2f> points_2d;
    ros::Rate loop_rate(30);
    int points_size = 0;
    Mat K = ( Mat_<double> ( 3,3 ) << 520.9, 0, 325.1, 0, 521.0, 249.7, 0, 0, 1 );
    Mat pose = Mat::zeros( 3, 1, CV_64FC1 );
    while ( ros::ok() ) {
        cap >> frame ;
        points.clear();
        matches = findmatch_ORB( preframe, frame, kp1, kp2 );
        pose_estimation_2d2d( kp1, kp2, matches, R, t );
        triangulation( kp1, kp2, matches, R, t, points );
        points_size = points.size();
        for ( KeyPoint p : kp1 ) {
            points_2d.push_back( p.pt );
        }
        bundleAdjustment(points, points_2d, K, R, t);

        cloud.width  = points_size;
        cloud.height = 1;
        cloud.points.resize(cloud.width * cloud.height);

        for (size_t i = 0; i < cloud.points.size(); ++i)
        {
            cloud.points[i].x = abs(points[i].x);
            cloud.points[i].y = abs(points[i].y);
            cloud.points[i].z = abs(points[i].z);
            // cout << points << endl;
            // cloud.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
            // cloud.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
            // cloud.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
            // cout << cloud.points[i].x <<endl;
        }
        pcl::toROSMsg(cloud, output);
        output.header.frame_id = "odom";
        pcl_pub.publish(output);

        pose = R * pose + t;
        geometry_msgs::Point tmp;
        tmp.x = pose.at<double>(0, 0);
        tmp.y = pose.at<double>(1, 0);
        tmp.z = pose.at<double>(2, 0);
        pose_pub.publish(tmp);

        preframe = frame;
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
