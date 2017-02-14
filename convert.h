#pragma once
#include <opencv2/core/core.hpp>

void computeAnglesFromMatrix(
                 cv::Mat R,
                 double & angle_x,
                 double & angle_y,
                 double & angle_z
                 );

cv::Mat computeMatrixFromAngles(
     double x,
     double y,
     double z);
