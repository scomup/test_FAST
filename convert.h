#pragma once
#include <opencv2/core/core.hpp>

#define PI 3.14159265258979

void computeAnglesFromMatrix(
                 cv::Mat R,
                 float & angle_x,
                 float & angle_y,
                 float & angle_z
                 );

cv::Mat computeMatrixFromAngles(
     float x,
     float y,
     float z);
