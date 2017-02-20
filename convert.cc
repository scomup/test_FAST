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

/*
void toEulerianAngle(const Quaterniond& q, double& roll, double& pitch, double& yaw)
{
	double ysqr = q.y() * q.y();

	// roll (x-axis rotation)
	double t0 = +2.0 * (q.w() * q.x() + q.y() * q.z());
	double t1 = +1.0 - 2.0 * (q.x() * q.x() + ysqr);
	roll = std::atan2(t0, t1);

	// pitch (y-axis rotation)
	double t2 = +2.0 * (q.w() * q.y() - q.z() * q.x());
	t2 = t2 > 1.0 ? 1.0 : t2;
	t2 = t2 < -1.0 ? -1.0 : t2;
	pitch = std::asin(t2);

	// yaw (z-axis rotation)
	double t3 = +2.0 * (q.w() * q.z() + q.x() * q.y());
	double t4 = +1.0 - 2.0 * (ysqr + q.z() * q.z());  
	yaw = std::atan2(t3, t4);
}

cv::Quaterniond toQuaternion(double pitch, double roll, double yaw)
{
	cv::Quaterniond q;
	double t0 = std::cos(yaw * 0.5);
	double t1 = std::sin(yaw * 0.5);
	double t2 = std::cos(roll * 0.5);
	double t3 = std::sin(roll * 0.5);
	double t4 = std::cos(pitch * 0.5);
	double t5 = std::sin(pitch * 0.5);

	q.w() = t0 * t2 * t4 + t1 * t3 * t5;
	q.x() = t0 * t3 * t4 - t1 * t2 * t5;
	q.y() = t0 * t2 * t5 + t1 * t3 * t4;
	q.z() = t1 * t2 * t4 - t0 * t3 * t5;
	return q;
}*/