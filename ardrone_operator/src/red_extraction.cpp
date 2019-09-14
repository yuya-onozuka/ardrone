#include <ardrone_operator/red_extraction.h>

cv::Mat RedExtraction::redExtraction(cv::Mat& image)
{
    int width = image.cols;
    int height = image.rows;
    
    cv::Mat convert_image = cv::Mat(cv::Size(width, height),CV_8UC1);

    double r_rate = 0.;

    for (int u = 0; u < width; u++){
        for (int v = 0; v < height; v++)
        {
            int B = (uchar)image.at<cv::Vec3b>(v,u)[0] + 1;
            int G = (uchar)image.at<cv::Vec3b>(v,u)[1] + 1;
            int R = (uchar)image.at<cv::Vec3b>(v,u)[2] + 1; 

            r_rate = R/(1.0*(R+G+B));
 
            if(r_rate>0.5)
            {
                convert_image.at<unsigned char>(v,u) = 255;
                // cv::cvtColor(ipt_image,red_image,CV_BGR2GRAY);
            }
            else
            { 
                convert_image.at<unsigned char>(v,u) = 0;
            }
        }
    }
    return convert_image;
}