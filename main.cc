#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ORBextractor.h"

#include <iostream>
const char filename_test[] = "/home/liu/workspace/test_FAST/right_image0000.png"; // 512x512 px 3 channels

using namespace cv;
using namespace ORB_SLAM;
int main()
{
	std::cout << "== Color image ==" << std::endl;

	Mat input,output;
	input = imread(filename_test);
    cvtColor(input, input, CV_RGB2GRAY);
    ORBextractor orb  =  ORBextractor();
    ORBextractor orb0  =  ORBextractor(1000,  1.2f, 8,  0, 20);

    std::vector<cv::KeyPoint> keys;
    cv::Mat descriptors;
 

  /*  
    Mat gb;
    GaussianBlur(input, gb, Size(5,5), 1);
    orb(gb,cv::Mat(),keys,descriptors);
    drawKeypoints(gb, keys, gb);
    imshow("GaussianBlur",gb);
    printf("GaussianBlur:%d\n",keys.size());

    Mat mb;
    medianBlur(input, mb, 5);
    orb(mb,cv::Mat(),keys,descriptors);
    drawKeypoints(mb, keys, mb);
    imshow("medianBlur",mb);
    printf("medianBlur:%d\n",keys.size());
*/
    Mat bi;
    bilateralFilter(input, bi, -1,30,5);
    orb(bi,cv::Mat(),keys,descriptors);
    drawKeypoints(bi, keys, bi);
    imshow("bilateralFilter",bi);
    printf("bilateralFilter:%d\n",keys.size());

    Mat bi0;
    bilateralFilter(input,bi0, -1,30,5);
    orb0(bi0,cv::Mat(),keys,descriptors);
    drawKeypoints(bi0, keys, bi0);
    imshow("bilateralFilter0",bi0);
    printf("bilateralFilter0:%d\n",keys.size());



/*
    orb(input,cv::Mat(),keys,descriptors);
    imwrite("original.png",input);
    drawKeypoints(input, keys, input);
    imshow("original",input);
    printf("original:%d\n",keys.size());

*/
    waitKey(0);

}