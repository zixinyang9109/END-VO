//
// Created by yzx on 9/7/21.
//

#include <opencv2/imgcodecs.hpp>
#include "myslam/common_include.h"

using namespace std;
using namespace cv;

int main(){


    string depth_path = "/media/yzx/Elements/Result_flow/Dataset2/keyframe_2/data/depth/00000.png";
    Mat depth = cv::imread(depth_path);
    int v_test = 398;
    int u_test = 446;
    double depth_scale = 200;


//
//    depth = depth /200;


    int rows = depth.rows;
    int cols = depth.cols;
    Mat depth_real = Mat::zeros(Size(cols,rows) , CV_64FC1);

    for (int v = 0; v < rows; v++)
        for (int u = 0; u < cols; u++) {
            double d = depth.ptr<unsigned short>(v)[u]; // 深度值
            cout<<"before scale "<<d<<endl;
            d = d/depth_scale;

            depth_real.ptr<double> (v)[u]= d;
            cout<<"after scale "<<d<<endl;
//
//            double new_d = depth_real.ptr<double>(v_test)[u_test]; // 深度值
//
//            cout<<"new depth_value "<< new_d<<endl;

            double d_o = depth.ptr<unsigned short>(v)[u]; // 深度值

            cout<<"depth_value "<< double(d_o)/depth_scale<<endl;

        }






}
