#include "tzj_slam/feature_extract.hpp"

using namespace cv;
using namespace std;

vector<DMatch> findmatch_ORB( cv::Mat& src1, cv::Mat& src2, vector<KeyPoint>& kp1,  vector<KeyPoint>& kp2 ) {
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("BruteForce");
    
    kp1 = get_keypoints( src1 );
    kp2 = get_keypoints( src2 );
    
    Ptr<ORB> extractor = ORB::create() ;
    cv::Mat descriptors1, descriptors2 ;
    extractor->compute( src1, kp1, descriptors1 );
    extractor->compute( src2, kp2, descriptors2 );

    vector<DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches, Mat() );

    // Mat img;
    // drawMatches(src1, kp1, src2, kp2, matches, img, Scalar::all(-1), Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    // imshow( "Test", img );

    return matches;
}

vector<KeyPoint> get_keypoints( const cv::Mat& frame ) {
    std::vector<KeyPoint> kp;

    // default in 2.4.9 is : ORB(700, 1.2f, 3, 31, 0);
    Ptr<ORB> detector = ORB::create(); // (500, 1.2f, 8, 31, 0); // default values of 2.3.1
    detector->detect(frame, kp);

    return kp;
}