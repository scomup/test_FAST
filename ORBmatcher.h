

#ifndef ORBMACHER_H
#define ORBMACHER_H


#include <opencv2/core/core.hpp>
#include "Frame.h"

using namespace cv;
using namespace ORB_SLAM;
using namespace std;
int SearchForInitialization(Frame &F1, Frame &F2, vector<cv::Point2f> &vbPrevMatched, vector<int> &vnMatches12, int windowSize);





#endif

