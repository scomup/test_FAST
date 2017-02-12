#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Frame.h"
#include <iostream>
const char filename_right[] = "/home/liu/workspace/test_FAST/right1.png"; // 512x512 px 3 channels
const char filename_left[] = "/home/liu/workspace/test_FAST/left1.png";   // 512x512 px 3 channels
using namespace cv;
using namespace ORB_SLAM;

int main()
{

    Mat right, left;
    Mat right_c, left_c;
    right_c = imread(filename_right);
    left_c = imread(filename_left);
    cvtColor(right_c, left, CV_RGB2GRAY);
    cvtColor(left_c, right, CV_RGB2GRAY);

    ORBextractor orb = ORBextractor(2000, 1.2f, 8, 1, 20);

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = 320;
    K.at<float>(1, 1) = 320;
    K.at<float>(0, 2) = 320.5;
    K.at<float>(1, 2) = 240.5;

    cv::Mat DistCoef(4, 1, CV_32F);
    DistCoef.at<float>(0) = 0;
    DistCoef.at<float>(1) = 0;
    DistCoef.at<float>(2) = 0;
    DistCoef.at<float>(3) = 0;

    Frame Frame_left = Frame(left, 0, &orb, K, DistCoef);
    Frame Frame_right = Frame(right, 0, &orb, K, DistCoef);
    std::vector<int> vnMatches12;
    std::vector<cv::Point2f> mvbPrevMatched;
    SearchForInitialization(Frame_left, Frame_right, mvbPrevMatched,vnMatches12,100  );
    for(int i = 0; i<vnMatches12.size();i++){
        if(vnMatches12[i] == -1)
            continue;
        cv::KeyPoint kp_l = Frame_left.mvKeys[i];
         cv::KeyPoint kp_r = Frame_right.mvKeys[vnMatches12[i]];
        line(left_c, kp_l.pt, kp_r.pt,Scalar( 0, 0, 255 ));
    }
    imshow("left",left_c);
    waitKey(0);
}


