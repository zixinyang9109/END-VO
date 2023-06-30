//
// Created by yzx on 9/2/21.
//


#include "myslam/common_include.h"
//#include "myslam/ORBextractor.h"
#include <iostream>
#include <vector>
#include<boost/format.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>
#include<string>
#include "myslam/camera.h"
//#include "myslam/geometry_cv.h"
#include "myslam/config.h"
#include "myslam/algorithm.h"
#include "myslam/frame.h"
#include "myslam/feature.h"
#include "myslam/map.h"
#include <Eigen/Core>
#include <vector>
#include <string>
#include <Eigen/Core>
#include "myslam/mappoint.h"
#include <pangolin/pangolin.h>
#include "numeric"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace std;
using namespace cv;
using namespace cv::ximgproc;


bool getcamera(std::vector<myslam::Camera::Ptr> &cameras_) {
    // read camera intrinsics and extrinsics
    auto fx1=myslam::Config::Get<double> ("camera1.fx")*0.5;
    auto fy1=myslam::Config::Get<double> ("camera1.fy")*0.5;
    auto cx1=myslam::Config::Get<double> ("camera1.cx")*0.5;
    auto cy1=myslam::Config::Get<double> ("camera1.cy")*0.5;

    auto fx2=myslam::Config::Get<double> ("camera2.fx")*0.5;
    auto fy2=myslam::Config::Get<double> ("camera2.fy")*0.5;
    auto cx2=myslam::Config::Get<double> ("camera2.cx")*0.5;
    auto cy2=myslam::Config::Get<double> ("camera2.cy")*0.5;

    auto tx=myslam::Config::Get<double> ("tx");
    auto ty=myslam::Config::Get<double> ("ty");
    auto tz=myslam::Config::Get<double> ("tz");

    Vec3 t1;
    Vec3 t2;
    t1 << 0, 0, 0;
    t2 << tx, ty, tz;

    myslam::Camera::Ptr new_camera0(new myslam::Camera(fx1, fy1, cx1, cy1,
                                                       t1.norm(), SE3(SO3(), t1)));
    cameras_.push_back(new_camera0);

    myslam::Camera::Ptr new_camera1(new myslam::Camera(fx2, fy2, cx2, cy2,
                                                       t2.norm(), SE3(SO3(), t2)));
    cameras_.push_back(new_camera1);

    return true;
}


void clahe(cv::Mat bgr_image,cv::Mat &image_clahe)
{
    // READ RGB color image and convert it to Lab
    //cv::Mat bgr_image = cv::imread("image.png");
    cv::Mat lab_image;
    cv::cvtColor(bgr_image, lab_image, CV_BGR2Lab);

    // Extract the L channel
    std::vector<cv::Mat> lab_planes(3);
    cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

    // apply the CLAHE algorithm to the L channel
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(4);
    cv::Mat dst;
    clahe->apply(lab_planes[0], dst);

    // Merge the the color planes back into an Lab image
    dst.copyTo(lab_planes[0]);
    cv::merge(lab_planes, lab_image);

    // convert back to RGB
    //cv::Mat image_clahe;
    cv::cvtColor(lab_image, image_clahe, CV_Lab2BGR);

    //    // display the results  (you might also want to see lab_planes[0] before and after).
    //    cv::imshow("image original", bgr_image);
    //    cv::imshow("image CLAHE", image_clahe);
    //    cv::waitKey();
}

void minmax(cv::Mat depth,string name){

    double min_index, max_index;
    minMaxIdx(depth, &min_index, &max_index);

    cout<< name<< endl;
    printf("  min index: %f, max index: %f\n", min_index, max_index);

}

inline float GetPixelValue(const cv::Mat &img, float x, float y) {
    // boundary check
    if (x < 0) x = 0;
    if (y < 0) y = 0;
    if (x >= img.cols) x = img.cols - 1;
    if (y >= img.rows) y = img.rows - 1;
    uchar *data = &img.data[int(y) * img.step + int(x)];
    float xx = x - floor(x);
    float yy = y - floor(y);
    return float(
            (1 - xx) * (1 - yy) * data[0] +
            xx * (1 - yy) * data[1] +
            (1 - xx) * yy * data[img.step] +
            xx * yy * data[img.step + 1]
            );
}


int main(){


    // read images
    string im1_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/image_0/00000.jpg";
    string im2_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/image_1/00000.jpg";
    //string depth_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
    string depth_path = "/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";
    string gt_depth_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";

    double depth_scale =200;//200;

    //"/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
    //"/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";

    Mat img1 = imread(im1_path);
    Mat color;
    Mat img2 = imread(im2_path);
    Mat depth = imread(depth_path);
    Mat gt_depth = imread(gt_depth_path);

    //clahe
    clahe(img1,img1);
    clahe(img2,img2);

    // gray
    Mat img1_gray,img2_gray;
    cv::cvtColor(img1, img1_gray, CV_BGR2GRAY);
    cv::cvtColor(img2, img2_gray, CV_BGR2GRAY);


    cv::Mat image_left_resized, image_right_resized, depth_resized, gt_depth_resized;
    cv::resize(img1_gray, image_left_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(img2_gray, image_right_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(depth, depth_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(gt_depth, gt_depth_resized, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    //    img1 = image_left_resized;
    //    color = img1;
    //    img2 = image_right_resized;

    minmax(depth_resized,"depth");

    //    cout<<SO3().matrix()<<endl;
    string config_file = "/home/yzx/CLionProjects/Dense_Tracking_and_Mapping/config/endoscope0103.yaml";
    myslam::Config::SetParameterFile ( config_file );

    // load camera intrinsics and extrinsics
    std::vector<myslam::Camera::Ptr> cameras_;
    getcamera(cameras_);
    myslam::Camera::Ptr camera_left_;
    myslam::Camera::Ptr camera_right_;

    camera_left_ = cameras_.at(0);
    camera_right_ = cameras_.at(1);

    // projection
    // 1. pixel2world
    // get pixel location
    // pixel2world
    //        auto px = camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
    //        kps_right.push_back(cv::Point2f(px[0], px[1]));


    // difference with optical flow
    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    int num = 5000;
    Ptr<GFTTDetector> detector = GFTTDetector::create(num, 0.01, 20); // maximum 500 keypoints
    detector->detect(image_left_resized, kp1);


    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);

//    for (auto i=0;i<pt1.size();i++){
//        int u = pt1[i].x;
//        int v = pt1[i].y;
//
//        double depth_pixel = depth_resized.ptr<unsigned short>(v)[u]/depth_scale;
//
//        if (depth_pixel>0){
//            Vec2 pixel_left(u,v);
//            auto pword_project = camera_left_->pixel2world(pixel_left,camera_left_->pose(),depth_pixel); // left 2 world
//            auto pixel_right_project = camera_right_->world2pixel(pword_project, camera_left_->pose()); // world 2 right
//            auto u_right = pixel_right_project(0,0);
//            auto v_right = pixel_right_project(1,0);
//            pt2.push_back(cv::Point2f(u_right,v_right));
//
//        }
//    }

    vector<uchar> status;
    vector<float> error;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(image_left_resized, image_right_resized, pt1, pt2, status, error);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv: " << time_used.count() << endl;

    //cout<<pt1[0]<<endl;
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};

    cout<<"pose left "<<camera_left_->pose().matrix()<<endl;
    cout<<"pose right "<<camera_right_->pose().matrix()<<endl;

    double diff_deep = 0;
    double diff_tri = 0;
    double diff_deep_tri = 0;
    int num_depth = 0;

    //compare depth
    for (int i =0;i<pt1.size();i++){
        if (status[i]){

            printf("current %d \n",i);

            int u = pt1[i].x;
            int v = pt1[i].y;

            Vec2 pixel_left(u,v);
            Vec2 pixel_right_track(pt2[i].x,pt2[i].y);

            // this means that src.at(i,j) is using (i,j) as (row,column) but Point(x,y) is using (x,y) as (column,row)
            double depth_pixel = depth_resized.ptr<unsigned short>(v)[u]/depth_scale;
            double gt_depth_pixel = gt_depth_resized.ptr<unsigned short>(v)[u]/depth_scale;

            std::vector<Vec3> points{
                camera_left_->pixel2camera(pixel_left),
                camera_right_->pixel2camera(pixel_right_track)
            };

            Vec3 pworld_tri = Vec3::Zero();
            myslam::triangulation(poses, points, pworld_tri);
            double depth_tri = pworld_tri(2,0);

            cout<< " depth_tri  "<< depth_tri <<endl;
            cout<<" depth_pixel "<< depth_pixel <<endl;
            cout<<" depth_gt "<< gt_depth_pixel <<endl;

            diff_deep = diff_deep + abs(gt_depth_pixel-depth_pixel);
            diff_tri = diff_tri + abs(gt_depth_pixel-depth_tri);
            diff_deep_tri = diff_deep_tri + abs(depth_pixel-depth_tri);
            num_depth++;

        }
    }

    cout<<" mean tri "<<diff_tri/num <<endl;
    cout<<" mean deep "<< diff_deep/num <<endl;
    cout<<" mean tri-deep "<< diff_deep/num <<endl;


}