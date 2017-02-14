#include "convert.h"

#define PI 3.14159265258979

void computeAnglesFromMatrix(
                 cv::Mat R,
                 double & angle_x,
                 double & angle_y,
                 double & angle_z
                 ){

  double threshold = 0.001;

  if(abs(R.at<double>(2,1) - 1.0) < threshold){ // R(2,1) = sin(x) = 1の時
    angle_x = PI / 2;
    angle_y = 0;
    angle_z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }else if(abs(R.at<double>(2,1) + 1.0) < threshold){ // R(2,1) = sin(x) = -1の時
    angle_x = - PI / 2;
    angle_y = 0;
    angle_z = atan2(R.at<double>(1,0), R.at<double>(0,0));
  }else{
    angle_x = asin(R.at<double>(2,1));
    angle_y = atan2(-R.at<double>(2,0), R.at<double>(2,2));
    angle_z = atan2(-R.at<double>(0,1), R.at<double>(1,1));
  }
}

cv::Mat computeMatrixFromAngles(
     double x,
     double y,
     double z){
  cv::Mat R = cv::Mat::zeros(3, 3, CV_32F);
  R.row(0).col(0) = cos(y)*cos(z) - sin(x)*sin(y)*sin(z);
  R.row(0).col(1) = -cos(x)*sin(z);
  R.row(0).col(2) = sin(y)*cos(z) + sin(x)*cos(y)*sin(z);
  R.row(1).col(0) = cos(y)*sin(z) + sin(x)*sin(y)*cos(z);
  R.row(1).col(1) = cos(x)*cos(z);
  R.row(1).col(2) = sin(y)*sin(z) - sin(x)*cos(y)*cos(z);
  R.row(2).col(0) = - cos(x)*sin(y);
  R.row(2).col(1) = sin(x);
  R.row(2).col(2) = cos(x)*cos(y);
  return R;
}