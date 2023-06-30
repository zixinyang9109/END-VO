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
    cout<<"max value: "<<max_index<<endl;
    cout<<"min value "<<min_index<<endl;

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


void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
            );

    pangolin::View &d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    while (pangolin::ShouldQuit() == false) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);

        glPointSize(2);
        glBegin(GL_POINTS);
        for (auto &p: pointcloud) {
            glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
            glVertex3d(p[0], p[1], p[2]);
        }
        glEnd();
        pangolin::FinishFrame();
        usleep(5000);   // sleep 5 ms
    }
    return;
}



int main(){


    // read images
    string im1_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/image_0/00000.jpg";
    string im2_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/image_1/00000.jpg";
    string gt_depth_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
    string depth_path = "/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";
    double depth_scale =200;//200;

    //"/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
    //"/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";

    Mat img1 = imread(im1_path);
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

    // projection left to right

    int rows = image_left_resized.rows;
    int cols = image_left_resized.cols;
    Mat img1_warp = Mat::zeros(Size(cols,rows) , CV_8UC1);
    Mat valid_mask =  Mat::zeros(Size(cols,rows) , CV_8UC1);
    int invalid_num = 0;
    // points for optical flow
    vector<Point2f> pt1,pt2;

    for (int u=0; u<cols;u++){
        for (int v=0;v<rows;v++){
            double depth_pixel = depth_resized.ptr<unsigned short>(v)[u]/depth_scale;
            if (depth_pixel>0){
                Vec2 pixel_left(u,v);
                auto pword_project = camera_left_->pixel2world(pixel_left,camera_left_->pose(),depth_pixel); // left 2 world
                auto pixel_right_project = camera_right_->world2pixel(pword_project, camera_left_->pose()); // world 2 right
                auto u_right = pixel_right_project(0,0);
                auto v_right = pixel_right_project(1,0);

                if ((u_right<cols) && (v_right<rows)){
                    auto pixel_value = GetPixelValue(image_right_resized,u_right,v_right);

                    pt1.push_back(cv::Point2f(u, v));
                    pt2.push_back(cv::Point2f(u_right,v_right));

                    img1_warp.at<uchar>(v,u) = pixel_value;
                    valid_mask.at<uchar>(v,u) = 255;
                }
                else{
                    invalid_num++;
                }
            }

        }

    }

    imshow("projected",img1_warp);
    imshow("img1",image_left_resized);
    imshow("valid mask",valid_mask);


    vector<uchar> status;
    vector<float> error;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(image_left_resized, image_right_resized, pt1, pt2, status, error);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);


    // triangullation
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    //Mat depth_Tri = Mat::zeros(Size(cols,rows) , CV_64F);
    double int_depth =0;
    Mat depth_Tri(rows,cols,CV_64F,int_depth); //Mat::zeros(Size(cols,rows) , CV_8UC1);
   // Mat depth_Tri(cols,rows,CV_64F,int_depth);
    //minmax(depth_Tri,"Triangulation depth");
    vector<double> depth_diff;

    //compare depth
    for (int i =0;i<pt1.size();i++){
        if (status[i]){

            int u = pt1[i].x;
            int v = pt1[i].y;

            Vec2 pixel_left(u,v);
//            if ((pt2[i].x<cols) && (pt2[i].y<rows))
//            {
            Vec2 pixel_right_track(pt2[i].x,pt2[i].y);
            //double depth_pixel = depth_resized.ptr<unsigned short>(v)[u]/depth_scale; // this means that src.at(i,j) is using (i,j) as (row,column) but Point(x,y) is using (x,y) as (column,row)

            std::vector<Vec3> points{
                camera_left_->pixel2camera(pixel_left),
                camera_right_->pixel2camera(pixel_right_track)
            };

            Vec3 pworld_tri = Vec3::Zero();
            myslam::triangulation(poses, points, pworld_tri);
            double depth_tri = pworld_tri(2,0);
            double depth_pixel = depth_resized.ptr<unsigned short>(v)[u]/depth_scale;

            auto pword_project = camera_left_->pixel2world(pixel_left,camera_left_->pose(),depth_tri); // left 2 world
            auto pixel_right_project = camera_right_->world2pixel(pword_project, camera_left_->pose()); // world 2 right

            double diff_value = abs(depth_pixel - depth_tri);

            depth_diff.push_back(diff_value);
            //printf("current %d \n",i);
//            cout<< " depth_tri  "<<depth_tri<<endl;
//            cout<<" depth_pixel "<<depth_pixel<<endl;
//            cout<<" depth_diff "<<depth_pixel - depth_tri<<endl;
//            depth_diff.push_back(depth_pixel - depth_tri);
//
//            cout<< " the difference x  " <<pixel_right_project[0] - pixel_right_track[0]<<endl;
//            cout<< " the difference y  " <<pixel_right_project[1] - pixel_right_track[1]<<endl;

            if (diff_value<5) depth_Tri.ptr<double>(v)[u] = depth_tri;
//            }


//

        }
    }

    //imshow("depth",depth_Tri);
    minmax(depth_Tri,"Triangulation depth");
    minmax(depth_resized,"Deep depth");
    imshow("filtered",depth_Tri);



//    double mean_mat =0;
//    int num_val = 0;
//    double min = 10000;
//    double max = -10000;
//
//    for (int i=0;i<depth_diff.size();i++){
//        mean_mat = mean_mat + depth_diff[i];
//        num_val++;
//        if(depth_diff[i]>max) max= depth_diff[i];
//        if(depth_diff[i]<min) min= depth_diff[i];
//    }
//
//    cout<<"the tot mat value "<<mean_mat<<endl;
//    cout<<"the num mat value "<<num_val<<endl;
//    cout<<"the min value "<<min<<endl;
//    cout<<"the max value "<<max<<endl;
//    cout<<"mean mat value "<<mean_mat/num_val<<endl;


    imwrite("/media/yzx/Elements/SCARED_RECONSTRUCTION/depth.png",depth_Tri);
//    double maxValue = *max_element(depth_diff.begin(),depth_diff.end());
//    double minValue = *min_element(depth_diff.begin(),depth_diff.end());
    Mat depth_tri = imread("/media/yzx/Elements/SCARED_RECONSTRUCTION/depth.png");
    minmax(depth_Tri,"Triangulation depth");
//    cout<<"maxValue "<<maxValue<<endl;
//    cout<<"minValue "<<minValue<<endl;

    double cx = camera_left_->cx_;
    double cy = camera_left_->cy_;
    double fx = camera_left_->fx_;
    double fy = camera_left_->fy_;
    double depthScale = 200;
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    //pointcloud.reserve(1000000);



    Mat depth1 = gt_depth_resized;
    Mat depth2 = depth_resized;// gt_depth_resized
    double diff=0;
    double diff_tri=0;
    int num=0;


    for (int u=0; u<cols;u++){
        for (int v=0;v<rows;v++){
            double depth_gt_pixel = depth1.ptr<unsigned short>(v)[u]/depth_scale;
            double depth_pred_pixel = depth2.ptr<unsigned short>(v)[u]/depth_scale;
            double depth_Tri_pixel = depth_Tri.ptr<double>(v)[u];

            if ( (depth_gt_pixel>0)&&(depth_pred_pixel>0)&&(depth_Tri_pixel>0))
            {
                cout<< "gt "<<depth_gt_pixel<<endl;
                cout<< "pred "<<depth_pred_pixel<<endl;
                diff = diff + abs(depth_gt_pixel - depth_pred_pixel);
                diff_tri = diff_tri + abs(depth_gt_pixel - depth_Tri_pixel);
            }
            num++;
        }

    }

    cout<<"the mean value "<< diff/num <<endl;
    cout<<"the mean value "<< diff_tri/num <<endl;


    cv::Mat color;
    cv::resize(img1, color, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);

    for (int v = 0; v < color.rows; v++)
        for (int u = 0; u < color.cols; u++) {
            double d = depth_Tri.ptr<double>(v)[u]; // 深度值
            if (d == 0) continue; // 为0表示没有测量到
            Eigen::Vector3d point;
            point[2] = double(d); // depthScale;
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;
            Eigen::Vector3d pointWorld = point;

            Vector6d p;
            p.head<3>() = pointWorld;
            p[5] = color.data[v * color.step + u * color.channels()];   // blue
            p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
            p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
            pointcloud.push_back(p);
        }

    cout << "点云共有" << pointcloud.size() << "个点." << endl;
    showPointCloud(pointcloud);
    waitKey(0);
    // Visualize( current_frame_,points_w,pointcloud);waitKey(0);




//    // optical flow
//    vector<KeyPoint> kp1;
//    int num = 5000;
//    Ptr<GFTTDetector> detector = GFTTDetector::create(num, 0.01, 20); // maximum 500 keypoints
//    detector->detect(image_left_resized, kp1);
//
//    vector<Point2f> pt1, pt2;
//    for (auto &kp: kp1) pt1.push_back(kp.pt);
//    vector<uchar> status;
//    vector<float> error;
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    cv::calcOpticalFlowPyrLK(image_left_resized, image_right_resized, pt1, pt2, status, error);
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "optical flow by opencv: " << time_used.count() << endl;


    // triangulation




}