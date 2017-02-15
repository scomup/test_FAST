#include "convert.h"


void computeAnglesFromMatrix(
                 cv::Mat R,
                 float& angle_x,
                 float& angle_y,
                 float& angle_z
                 ){

  float threshold = 0.001;

  if(abs(R.at<float>(2,1) - 1.0) < threshold){ // R(2,1) = sin(x) = 1の時
    angle_x = PI / 2;
    angle_y = 0;
    angle_z = atan2(R.at<float>(1,0), R.at<float>(0,0));
  }else if(abs(R.at<float>(2,1) + 1.0) < threshold){ // R(2,1) = sin(x) = -1の時
    angle_x = - PI / 2;
    angle_y = 0;
    angle_z = atan2(R.at<float>(1,0), R.at<float>(0,0));
  }else{
    angle_x = asin(R.at<float>(2,1));
    angle_y = atan2(-R.at<float>(2,0), R.at<float>(2,2));
    angle_z = atan2(-R.at<float>(0,1), R.at<float>(1,1));
  }
}

cv::Mat computeMatrixFromAngles(
     float x,
     float y,
     float z){
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