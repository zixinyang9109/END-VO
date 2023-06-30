/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "myslam/frame.h"

#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

namespace myslam {
    // 构造函数
Frame::Frame(long id, double time_stamp, const SE3 &pose, const Mat &left, const Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}



Frame::Ptr Frame::CreateFrame() {
    static long factory_id = 0;
    Frame::Ptr new_frame(new Frame);
    new_frame->id_ = factory_id++;
    return new_frame;
}

void Frame::SetKeyFrame() {
    static long keyframe_factory_id = 0;
    is_keyframe_ = true;
    keyframe_id_ = keyframe_factory_id++;
}

void Frame::SetCoverRegion(std::vector<cv::Point2f> pt1,std::vector<uchar> status) {


    // divide image into patches
    int rows = left_img_.rows;
    int cols = left_img_.cols;
    Mat patches = Mat::zeros(cv::Size(cols,rows) , CV_8UC1);

    int num_patches = 16;
    int h_patch = rows/num_patches;
    int w_patch = cols/num_patches;
    double label_num = num_patches*num_patches;
    double init_label = 0;

    for (int v_i=0;v_i<num_patches;v_i++) {
        for (int u_i = 0; u_i < num_patches; u_i++) {
            int u = u_i * w_patch;
            int v = v_i * h_patch;
            patches(cv::Rect(u, v, w_patch, h_patch)).setTo(init_label);
            init_label++;
        }
    }

    // create mask where keypoints exist
    Mat point_mat = Mat::zeros(cv::Size(cols,rows) , CV_8UC1);

    for (int i = 0; i < pt1.size(); i++) {
        if (status[i]) {
            int u  = pt1[i].x;
            int v = pt1[i].y;
            point_mat.ptr<unsigned int>(v)[u]=255;
//            std::cout<<"u"<<u<<std::endl;
//            std::cout<<"v"<<u<<std::endl;

        }
    }

//    cv::imshow("point mat",point_mat);
//    cv::waitKey(0);


    //
    double num_val_patch = 0;

//    Mat val_mask = Mat::zeros(cv::Size(cols,rows) , CV_8UC1);

    for (int label_index=0; label_index < label_num; label_index++){
        Mat sel_label = patches == label_index;

        double valid_label = (point_mat).dot(sel_label);
        //std::cout<< "count result  "<<valid_label<<std::endl;
        if (valid_label>0){

            num_val_patch++;
            //valid_index.push_back(label_index);
            //std::cout<< "valid_index  "<<label_index<<std::endl;
        }
    }

    covered_region_ = 100*num_val_patch/label_num;



}

}
