#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <iostream>
#include <fstream>

#include "ORBextractor.h"
#include "ORBmatcher.h"
#include "Frame.h"
#include "convert.h"
#include <limits>
#include <math.h> 


constexpr double epsilon = std::numeric_limits<double>::epsilon();

using namespace cv;
using namespace ORB_SLAM;


int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        std::cout << "Usage: ./test path_to_settings(absolute or relative to package directory)" << std::endl;
        return 1;
    }

    cv::FileStorage fsSettings(argv[1], cv::FileStorage::READ);


    if (!fsSettings.isOpened())
    {
        std::cout <<"Wrong path to settings." << std::endl;
        return 1;
    }

    std::string left_path, right_path, out_path;
    fsSettings["left.path"] >> left_path;
    fsSettings["right.path"] >> right_path;
    fsSettings["out.path"] >> out_path;


    std::ofstream ofs(out_path);

    Mat right, left;
    Mat right_c, left_c;

    right_c = imread(right_path);
    left_c = imread(left_path);
    if(right_c.empty() || left_c.empty())
    {
        std::cout <<"Wrong image path to settings, Please check your yaml file." << std::endl;
        return 1;
    }

    cvtColor(right_c, right, CV_RGB2GRAY);
    cvtColor(left_c, left, CV_RGB2GRAY);

    ORBextractor orb = ORBextractor(2000, 1.2f, 8, 1, 20);

    cv::Mat K_l = cv::Mat::eye(3, 3, CV_32F);
    K_l.at<float>(0, 0) = fsSettings["left.Camera.fx"];
    K_l.at<float>(1, 1) = fsSettings["left.Camera.fy"];
    K_l.at<float>(0, 2) = fsSettings["left.Camera.cx"];
    K_l.at<float>(1, 2) = fsSettings["left.Camera.cy"];

    cv::Mat DistCoef_l(4, 1, CV_32F);
    DistCoef_l.at<float>(0) = fsSettings["left.Camera.k1"];
    DistCoef_l.at<float>(1) = fsSettings["left.Camera.k2"];
    DistCoef_l.at<float>(2) = fsSettings["left.Camera.p1"];
    DistCoef_l.at<float>(3) = fsSettings["left.Camera.p2"];

    cv::Mat K_r = cv::Mat::eye(3, 3, CV_32F);
    K_r.at<float>(0, 0) = fsSettings["right.Camera.fx"];
    K_r.at<float>(1, 1) = fsSettings["right.Camera.fy"];
    K_r.at<float>(0, 2) = fsSettings["right.Camera.cx"];
    K_r.at<float>(1, 2) = fsSettings["right.Camera.cy"];

    cv::Mat DistCoef_r(4, 1, CV_32F);
    DistCoef_r.at<float>(0) = fsSettings["right.Camera.k1"];
    DistCoef_r.at<float>(1) = fsSettings["right.Camera.k2"];
    DistCoef_r.at<float>(2) = fsSettings["right.Camera.p1"];
    DistCoef_r.at<float>(3) = fsSettings["right.Camera.p2"];

    cv::Mat DistCoef  = cv::Mat::zeros(4, 1, CV_32F);
    cv::Mat newK_l;
    cv::Mat newK_r;
    cv::Mat R_;
    cv::Mat proj_r = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0,  0, 0, 1);;
    cv::Mat left_rect;
    cv::Mat right_rect;
    cv::Mat mapx_l, mapy_l;
	cv::Mat mapx_r, mapy_r;

    cv::initUndistortRectifyMap(K_l, DistCoef_l, R_, K_l, left.size(), CV_32FC1, mapx_l, mapy_l);
    cv::initUndistortRectifyMap(K_r, DistCoef_r, R_, K_r, left.size(), CV_32FC1, mapx_r, mapy_r);
    cv::remap(left,left_rect,mapx_l,mapy_l,cv::INTER_LINEAR);
    cv::remap(right,right_rect,mapx_r,mapy_r,cv::INTER_LINEAR);
    cv::remap(left_c,left_c,mapx_l,mapy_l,cv::INTER_LINEAR);
    cv::remap(right_c,right_c,mapx_r,mapy_r,cv::INTER_LINEAR);


    Frame Frame_left = Frame(left_rect, 0, &orb, K_l, DistCoef);
    Frame Frame_right = Frame(right_rect, 0, &orb, K_l, DistCoef);

    std::vector<int> vnMatches12;
    std::vector<cv::Point2f> vbPrevMatched;
    SearchForInitialization(Frame_left, Frame_right, vbPrevMatched, vnMatches12, 100);

    //int c= 0;
    for (size_t i = 0; i < vnMatches12.size(); i++)
    {
        if (vnMatches12[i] == -1)
            continue;
        //c++;
        cv::KeyPoint kp_l = Frame_left.mvKeys[i];
        cv::KeyPoint kp_r = Frame_right.mvKeys[vnMatches12[i]];
        line(left_c, kp_l.pt, kp_r.pt, Scalar(0, 0, 255));
        line(right_c, kp_l.pt, kp_r.pt, Scalar(0, 0, 255));
        circle(left_c, kp_l.pt, 2, Scalar(0, 255, 0));
        circle(right_c, kp_r.pt, 2, Scalar(0, 255, 0));
    }
    //cv::drawKeypoints(left_c, Frame_left.mvKeys, left_c, cv::Scalar::all(-1) );
    //printf("all:%d\n Matches:%d\n",vnMatches12.size(),c);

    //pose:original 
    cv::Mat Tcw_left = cv::Mat::eye(4, 4, CV_32F);
    cv::Mat Tcw_right = cv::Mat::eye(4, 4, CV_32F);
    Tcw_right.at<float>(0, 3) = fsSettings["right.trans.x"];
    Tcw_right.at<float>(1, 3) = fsSettings["right.trans.y"];
    Tcw_right.at<float>(2, 3) = fsSettings["right.trans.z"];

    cv::Mat Tcw1 = Tcw_left;
    cv::Mat Rcw1 = Tcw_left.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw1 = Tcw_left.rowRange(0, 3).col(3);
    

    cv::Mat Tcw2 = Tcw_right;
    cv::Mat Rcw2 = Tcw_right.rowRange(0, 3).colRange(0, 3);
    cv::Mat tcw2 = Tcw_right.rowRange(0, 3).col(3);

    cv::Mat Rwc1 = Rcw1.t();

    const float fx1 = Frame_right.fx;
    const float fy1 = Frame_right.fy;
    const float cx1 = Frame_right.cx;
    const float cy1 = Frame_right.cy;
    const float invfx1 = 1.0f / fx1;
    const float invfy1 = 1.0f / fy1;
    {
        float x_angle, y_angle, z_angle;
        cv::Mat Rwc_left = Tcw_left.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc_left = -Rwc_left * Tcw_left.rowRange(0, 3).col(3);
        computeAnglesFromMatrix(Rwc_left, x_angle, y_angle, z_angle);
        ofs << "camera:" << twc_left.at<float>(0) << "," << twc_left.at<float>(1) << "," << twc_left.at<float>(2)
            << "," << x_angle << "," << y_angle << "," << z_angle << std::endl;

        cv::Mat Rwc_right = Tcw_right.rowRange(0, 3).colRange(0, 3).t();
        cv::Mat twc_right = -Rwc_right * Tcw_right.rowRange(0, 3).col(3);
        computeAnglesFromMatrix(Rwc_right, x_angle, y_angle, z_angle);
        ofs << "camera:" << twc_right.at<float>(0) << "," << twc_right.at<float>(1) << "," << twc_right.at<float>(2)
            << "," << x_angle << "," << y_angle << "," << z_angle << std::endl;
    }

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

        //if (x3D.at<float>(3) == 0)
         //   continue;
        // Euclidean coordinates
        x3D = x3D.rowRange(0, 3) / x3D.at<float>(3);
        cv::Mat x3Dt = x3D.t();
        //Check triangulation in front of cameras
        float z1 = Rcw1.row(2).dot(x3Dt) + tcw1.at<float>(2);
        if (z1 <= 0){
            continue;
            std::cout<<"1"<<std::endl;

        }
        ofs <<"point:"<<x3Dt.at<float>(0)<<","<< x3Dt.at<float>(1)<<","<< x3Dt.at<float>(2)<<std::endl;
    }
    ofs.close();
    imshow("left", left_c);
    imshow("right", right_c);
    waitKey(0);
}

