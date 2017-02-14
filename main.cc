#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Frame.h"
#include "convert.h"

const char filename_right[] = "/home/liu/workspace/test_FAST/right.jpg"; // 512x512 px 3 channels
const char filename_left[] = "/home/liu/workspace/test_FAST/left.jpg";   // 512x512 px 3 channels
using namespace cv;
using namespace ORB_SLAM;

int main()
{
    std::ofstream ofs( "mapPoint.txt" );
    Mat right, left;
    Mat right_c, left_c;
    right_c = imread(filename_right);
    left_c = imread(filename_left);
    cvtColor(right_c, right, CV_RGB2GRAY);
    cvtColor(left_c, left, CV_RGB2GRAY);

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
    std::vector<cv::Point2f> vbPrevMatched;
    SearchForInitialization(Frame_left, Frame_right, vbPrevMatched, vnMatches12, 100);
    for (size_t i = 0; i < vnMatches12.size(); i++)
    {
        if (vnMatches12[i] == -1)
            continue;
        cv::KeyPoint kp_l = Frame_left.mvKeys[i];
        cv::KeyPoint kp_r = Frame_right.mvKeys[vnMatches12[i]];
        line(left_c, kp_l.pt, kp_r.pt, Scalar(0, 0, 255));
        line(right_c, kp_l.pt, kp_r.pt, Scalar(0, 0, 255));
        circle(left_c, kp_l.pt, 2, Scalar(0, 255, 0));
        circle(right_c, kp_r.pt, 2, Scalar(0, 255, 0));

    }
    cv::Mat Tcw_left = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Tcw_right = cv::Mat::eye(4, 4, CV_32F);

    cv::Mat R = computeMatrixFromAngles(0,0,-1.507);
    cout <<R<<endl;
    //Tcw_left.at<float>(0, 3) = -1.0;
    //Tcw_right.at<float>(0, 3) = -1.1;
    Tcw_right.at<float>(1, 3) = -0.1;
    //Tcw_left.rowRange(0, 3).colRange(0, 3) = R;
    //Tcw_right.rowRange(0, 3).colRange(0, 3) = R;
    R.copyTo(Tcw_left.rowRange(0, 3).colRange(0, 3));
    R.copyTo(Tcw_right.rowRange(0, 3).colRange(0, 3));
    std::cout <<"Tcw_right"<< Tcw_right << std::endl;
    std::cout <<"Tcw_left"<< Tcw_left << std::endl;
    cv::Mat Tcw1 = Tcw_left;
    cv::Mat Rcw1 = Tcw_left.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw1 = Tcw_left.rowRange(0, 3).col(3);

    cv::Mat Tcw2 = Tcw_right;
    cv::Mat Rcw2 = Tcw_right.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw2 = Tcw_right.rowRange(0, 3).col(3);

    //Rcw = Rcw2 * Rcw1.t();
    //tcw = tcw1 - tcw2;

    cv::Mat Rwc1 = Rcw1.t();

    const float fx1 = Frame_right.fx;
    const float fy1 = Frame_right.fy;
    const float cx1 = Frame_right.cx;
    const float cy1 = Frame_right.cy;
    const float invfx1 = 1.0f / fx1;
    const float invfy1 = 1.0f / fy1;

    for (size_t i = 0; i < vnMatches12.size(); i++)
    {
        if (vnMatches12[i] == -1)
            continue;
        const cv::KeyPoint kp1 = Frame_left.mvKeysUn[i];
        const cv::KeyPoint kp2 = Frame_right.mvKeysUn[vnMatches12[i]];
        //float disparity = sqrt(kp1.pt.x * kp1.pt.x + kp1.pt.y * kp1.pt.y) - sqrt(kp2.pt.x * kp2.pt.x + kp2.pt.y * kp2.pt.y);
        cv::Mat xn1 = (cv::Mat_<float>(3, 1) << (kp1.pt.x - cx1) * invfx1, (kp1.pt.y - cy1) * invfy1, 1.0);
        cv::Mat xn2 = (cv::Mat_<float>(3, 1) << (kp2.pt.x - cx1) * invfx1, (kp2.pt.y - cy1) * invfy1, 1.0);
        // Linear Triangulation Method
        cv::Mat A(4, 4, CV_32F);
        A.row(0) = xn1.at<float>(0) * Tcw1.row(2) - Tcw1.row(0);
        A.row(1) = xn1.at<float>(1) * Tcw1.row(2) - Tcw1.row(1);
        A.row(2) = xn2.at<float>(0) * Tcw2.row(2) - Tcw2.row(0);
        A.row(3) = xn2.at<float>(1) * Tcw2.row(2) - Tcw2.row(1);
        cv::Mat w, u, vt;
        cv::SVD::compute(A, w, u, vt, cv::SVD::MODIFY_A | cv::SVD::FULL_UV);
        cv::Mat x3D = vt.row(3).t();

        if (x3D.at<float>(3) == 0)
            continue;
        // Euclidean coordinates
        x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
        cv::Mat x3Dt = x3D.t();
        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
        if (z1 <= 0)
        //    continue;
        std::cout << x3Dt << std::endl;
        ofs << x3Dt.at<float>(0)<<","<< x3Dt.at<float>(1)<<","<< x3Dt.at<float>(2)<<std::endl;
    }
    ofs.close();

    imshow("left", left_c);
    imshow("right", right_c);
    waitKey(0);
}
