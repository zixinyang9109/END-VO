//
// Created by yzx on 9/9/21.
//
#include "myslam/common_include.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/video.hpp>
using namespace cv;
using namespace std;

int main(){


    // read images
    string im1_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION_rectify_black/Dataset1/keyframe_2/data/image_0/00000.jpg";
    string im2_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION_rectify_black/Dataset1/keyframe_2/data/image_1/00000.jpg";
    string depth_path = "/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/rec_depth/00000.png";

    Mat img1 = imread(im1_path);
    Mat color;
    Mat img2 = imread(im2_path);
    Mat depth = imread(depth_path);
    double depthScale = 200;


    // gray
    Mat img1_gray,img2_gray;
    cv::cvtColor(img1, img1_gray, CV_BGR2GRAY);
    cv::cvtColor(img2, img2_gray, CV_BGR2GRAY);


    cv::Mat image_left_resized, image_right_resized, depth_resized;
    cv::resize(img1_gray, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(img2_gray, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(depth, depth_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);


    Mat flow(img1.size(), CV_32FC2);
    calcOpticalFlowFarneback(image_left_resized, image_right_resized, flow, 0.5, 3, 15, 3, 5, 1.2, 0);

    // visualization
    Mat flow_parts[2];
    split(flow, flow_parts);
    Mat magnitude, angle, magn_norm;
    cartToPolar(flow_parts[0], flow_parts[1], magnitude, angle, true);
    normalize(magnitude, magn_norm, 0.0f, 1.0f, NORM_MINMAX);
    angle *= ((1.f / 360.f) * (180.f / 255.f));
    //build hsv image
    Mat _hsv[3], hsv, hsv8, bgr;
    _hsv[0] = angle;
    _hsv[1] = Mat::ones(angle.size(), CV_32F);
    _hsv[2] = magn_norm;
    merge(_hsv, 3, hsv);
    hsv.convertTo(hsv8, CV_8U, 255.0);
    cvtColor(hsv8, bgr, COLOR_HSV2BGR);
    imshow("frame2", bgr);
    waitKey(0);








}
