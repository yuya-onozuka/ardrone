#ifndef RED_EXTRACTION_H_
#define RED_EXTRACTION_H_

#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

class RedExtraction
{
public:
    static cv::Mat extractRedFrom(cv::Mat& image);
};

#endif // RED_EXTRACTION_H_