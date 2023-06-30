//
// Created by yzx on 9/6/21.
//

//
// Created by yzx on 8/9/21.
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
#include <cmath>
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
typedef Eigen::Matrix<double, 6, 1> Vector6d;
using namespace std;
using namespace cv;
using namespace cv::ximgproc;

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

//bool getcamera(std::vector<myslam::Camera::Ptr> &cameras_) {
//    // read camera intrinsics and extrinsics
//    string dataset_path_ = "/media/yzx/Elements/SCARED_RECONSTRUCTION_rectify_black/Dataset2/keyframe_2/data";
//    ifstream fin(dataset_path_ + "/calib.txt");
//    if (!fin) {
//        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
//        return false;
//    }
//
//    for (int i = 0; i < 2; ++i) {
//        char camera_name[3];
//        cout<<i<<endl;
//        for (int k = 0; k < 3; ++k) {
//            fin >> camera_name[k];
//        }
//        double projection_data[12];
//        for (int k = 0; k < 12; ++k) {
//            fin >> projection_data[k];
//        }
//
//        Mat33 K;
//        K << projection_data[0], projection_data[1], projection_data[2],
//        projection_data[4], projection_data[5], projection_data[6],
//        projection_data[8], projection_data[9], projection_data[10];
//
//        Vec3 t;
//        t << projection_data[3], projection_data[7], projection_data[11];
//        cout<<"before"<<t<<endl;
//        t = K.inverse() * t;
//        cout<<"after"<<t<<endl;
//        K = K * 0.5;
//
//        myslam::Camera::Ptr new_camera(new myslam::Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
//                                          t.norm(), SE3(SO3(), t)));
//        cout<<"t norm"<<t.norm()<<endl;
//
//        cameras_.push_back(new_camera);
//        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
//    }
//    fin.close();
//
//    return true;                                       t.norm(), SE3(SO3(), t)));
//        cout<<"t norm"<<t.norm()<<endl;
//
//        cameras_.push_back(new_camera);
//        LOG(INFO) << "Camera " << i << " extrinsics: " << t.transpose();
//    }
//    fin.close();
//
//    return true;
//}

bool getcamera(std::vector<myslam::Camera::Ptr> &cameras_) {
    // read camera intrinsics and extrinsics
//    auto fx1=myslam::Config::Get<double> ("camera1.fx")*0.5;
//    auto fy1=myslam::Config::Get<double> ("camera1.fy")*0.5;
//    auto cx1=myslam::Config::Get<double> ("camera1.cx")*0.5;
//    auto cy1=myslam::Config::Get<double> ("camera1.cy")*0.5;
//
//    auto fx2=myslam::Config::Get<double> ("camera2.fx")*0.5;
//    auto fy2=myslam::Config::Get<double> ("camera2.fy")*0.5;
//    auto cx2=myslam::Config::Get<double> ("camera2.cx")*0.5;
//    auto cy2=myslam::Config::Get<double> ("camera2.cy")*0.5;
//
//    auto tx=myslam::Config::Get<double> ("tx");
//    auto ty=myslam::Config::Get<double> ("ty");
//    auto tz=myslam::Config::Get<double> ("tz");
//
//    Vec3 t1;
//    Vec3 t2;
//    t1 << 0, 0, 0;
//    t2 << tx, ty, tz;
//
//
//    myslam::Camera::Ptr new_camera0(new myslam::Camera(fx1, fy1, cx1, cy1,
//                                                       t1.norm(), SE3(SO3(), t1)));
//    cameras_.push_back(new_camera0);
//
//    myslam::Camera::Ptr new_camera1(new myslam::Camera(fx2, fy2, cx2, cy2,
//                                                       t2.norm(), SE3(SO3(), t2)));
//    cameras_.push_back(new_camera1);


    auto fx1=myslam::Config::Get<double> ("camera1.fx");
    auto fy1=myslam::Config::Get<double> ("camera1.fy");
    auto cx1=myslam::Config::Get<double> ("camera1.cx");
    auto cy1=myslam::Config::Get<double> ("camera1.cy");

    auto fx2=myslam::Config::Get<double> ("camera2.fx");
    auto fy2=myslam::Config::Get<double> ("camera2.fy");
    auto cx2=myslam::Config::Get<double> ("camera2.cx");
    auto cy2=myslam::Config::Get<double> ("camera2.cy");

    auto tx=myslam::Config::Get<double> ("tx");
    auto ty=myslam::Config::Get<double> ("ty");
    auto tz=myslam::Config::Get<double> ("tz");

    Vec3 t1;
    Vec3 t2;

    t1 << 0, 0, 0;
    t2 << tx, ty, tz;

    Mat33 K1;
    K1 <<fx1, 0, cx1, 0, fy1, cy1, 0, 0, 1;

    Mat33 K2;
    K2 <<fx2, 0, cx2, 0, fy2, cy2, 0, 0, 1;

    //t2 = K.inverse() * t2;
    K1 = K1 * 0.5;
    K2 = K2 * 0.5;

    myslam::Camera::Ptr new_camera0(new myslam::Camera(K1(0, 0), K1(1, 1), K1(0, 2), K1(1, 2),
                                               t1.norm(), SE3(SO3(), t1)));
    LOG(INFO) << "Camera " << 0 << " intrinsics: " << new_camera0->K();
    LOG(INFO) << "Camera " << 0 << " extrinsics: " << t1;

    cameras_.push_back(new_camera0);

    myslam::Camera::Ptr new_camera1(new myslam::Camera(K2(0, 0), K2(1, 1), K2(0, 2), K2(1, 2),
                                               t2.norm(), SE3(SO3(), t2)));

    LOG(INFO) << "Camera " << 1 << " intrinsics: " << new_camera1->K();
    LOG(INFO) << "Camera " << 1 << " extrinsics: " << t2;

    cameras_.push_back(new_camera1);

    return true;
}

void DrawFrame(SE3 Twc, const float* color) {
//SE3 Twc = frame->Pose().inverse();
    const float sz = 1.0;
    const int line_width = 2.0;
    const float fx = 400;
    const float fy = 400;
    const float cx = 512;
    const float cy = 384;
    const float width = 1080;
    const float height = 768;

    glPushMatrix();

    Sophus::Matrix4f m = Twc.matrix().template cast<float>();
    glMultMatrixf((GLfloat*)m.data());

    if (color == nullptr) {
        glColor3f(1, 0, 0);
    } else
        glColor3f(color[0], color[1], color[2]);

    glLineWidth(line_width);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(0, 0, 0);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

    glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
    glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

    glEnd();
    glPopMatrix();
}

void FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera,SE3 Twc) {
//SE3 Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

void DrawMapPoints(std::unordered_map<unsigned long,Vec3> points_w) {
    const float red[3] = {1.0, 0, 0};

    glPointSize(4);
    glBegin(GL_POINTS);
    for (auto& landmark : points_w) {
        auto pos = landmark.second;
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

void showDepth(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p: pointcloud) {
        glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
        glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();

}

void Visualize( myslam::Frame::Ptr current_frame_,std::unordered_map<unsigned long,Vec3> points_w,const vector<Vector6d, Eigen::aligned_allocator<Vector6d>> &pointcloud) {
    pangolin::CreateWindowAndBind("MySLAM", 1024, 768);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::OpenGlRenderState vis_camera(
            pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
            pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& vis_display =
            pangolin::CreateDisplay()
                    .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                    .SetHandler(new pangolin::Handler3D(vis_camera));

    const float blue[3] = {0, 0, 1};
    const float green[3] = {0, 1, 0};

    while (!pangolin::ShouldQuit() ) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);


        // create current camera in 3D， require camera pose
        SE3 Twc = current_frame_->Pose().inverse();
        DrawFrame(Twc, green);
        FollowCurrentFrame(vis_camera,Twc);
        // show points on image 2D


        cv::Mat img_out;
        cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
        for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
            auto feat = current_frame_->features_left_[i];
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                       2);

        }
        cv::imshow("image", img_out);
        cv::waitKey(1);

        // draw landmarks

        DrawMapPoints(points_w);
        showDepth(pointcloud);


        pangolin::FinishFrame();
        usleep(5000);
    }

// LOG(INFO) << "Stop viewer";
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

void slc(Mat img, Mat &labels, Mat &mask){
    int region_size = 30;
    int ruler = 5;//Chooses the enforcement of superpixel smoothness factor of superpixel. The bigger, the square looks like.
    int min_element_size = 20;
    int num_iterations = 3;
    Mat img_blur;

    cv::GaussianBlur(img,img_blur,Size(3,3),3);

    //Mat converted;

    //cvtColor(img_blur, converted, COLOR_BGR2HSV);
    Ptr<cv::ximgproc::SuperpixelSLIC> slic = createSuperpixelSLIC(img,cv::ximgproc::SLIC,region_size,float(ruler));

    slic->iterate(num_iterations);
    if (min_element_size>0)
        slic->enforceLabelConnectivity(min_element_size);

    slic->getLabels(labels);
    slic->getLabelContourMask(mask);

}

void val_mask( vector<Point2f> pt1, Mat img1,  vector<uchar> status, Mat& valid_mask){

    // get slc segmentation
    Mat mask,labels;
    slc(img1,labels, mask);
    cv::imshow("mask", mask);
    cv::imshow("labels", labels);

    double min_index, max_index;
    minMaxIdx(labels, &min_index, &max_index);

    printf("min index: %f, max index: %f\n", min_index, max_index);

    int rows = img1.rows;
    int cols = img1.cols;
    Mat img1_CV_mask = Mat::zeros(Size(cols,rows) , CV_8UC1);


    for (int i = 0; i < pt1.size(); i++) {
        if (status[i]) {
            cv::circle(img1_CV_mask, pt1[i], 2, cv::Scalar(250), 2);
            //cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }

    //    Mat val_mask = Mat::zeros(Size(cols,rows) , CV_8UC1);

    for (int label_index=0; label_index < max_index; label_index++){
        Mat sel_label = labels == label_index;
        double valid_label = (img1_CV_mask/255.0).dot(sel_label/255);
        //cout<< "valid_label  "<<valid_label<<endl;
        if (valid_label>0){
            valid_mask = valid_mask + sel_label;
            //valid_index.push_back(label_index);
            //cout<< "valid_index  "<<label_index<<endl;
        }
    }



}


int main(){

    //    myslam::Map::Ptr map_ = nullptr;
    //    myslam::Viewer::Ptr viewer_ = nullptr;
    //
    //    map_ = myslam::Map::Ptr(new myslam::Map);
    //    viewer_ = myslam::Viewer::Ptr(new myslam::Viewer);
    //    viewer_->SetMap(map_);


    // read images
    string im1_path = "/media/yzx/Elements/SCARED_related/SCARED_RECONSTRUCTION_rectify_black/Dataset1/keyframe_2/data/image_0/00000.jpg";
    string im2_path = "/media/yzx/Elements/SCARED_related/SCARED_RECONSTRUCTION_rectify_black/Dataset1/keyframe_2/data/image_1/00000.jpg";
    string depth_path = "/media/yzx/Elements/SCARED_related/Result_flow/Dataset1/keyframe_2/data/rec_depth/00000.png";

//    string im1_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/image_0/00000.jpg";
//    string im2_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/image_1/00000.jpg";
//    string depth_path = "/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";

    //"/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
//    string im1_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/left/00000.jpg";
//    string im2_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/Dataset1/keyframe_2/data/right/00000.jpg";
//    string depth_path = "/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";
    //string depth_path = "/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
    //"/media/yzx/Elements/SCARED_RECONSTRUCTION/GT_DEPTH/1.png";
    //"/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00000.png";

    Mat img1 = imread(im1_path);
    Mat color;
    Mat img2 = imread(im2_path);
    Mat depth = imread(depth_path);
    double depthScale = 200;


    //clahe
    clahe(img1,img1);
    clahe(img2,img2);

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

    img1 = image_left_resized;
    color = img1;
    img2 = image_right_resized;
    depth = depth_resized;

    //    imshow("depth",depth);
    //    waitKey(0);
    double min_index, max_index;
    minMaxIdx(depth, &min_index, &max_index);

    printf(" Depth min index: %f, max index: %f\n", min_index, max_index);


    //    cout<<SO3().matrix()<<endl;
    string config_file = "/home/yzx/CLionProjects/Dense_Tracking_and_Mapping/config/endoscope0103.yaml";
    myslam::Config::SetParameterFile ( config_file );

    auto v_min= myslam::Config::Get<int> ("v_min");
    auto v_max = v_min + myslam::Config::Get<int> ("v_h");

    auto u_min = myslam::Config::Get<int> ("u_min");
    auto u_max = u_min + myslam::Config::Get<int> ("u_w");

    v_min = v_min/2;
    v_max = v_max/2;
    u_min = u_min/2;
    u_max = u_max/2;

    // load camera intrinsics
    std::vector<myslam::Camera::Ptr> cameras_;
    getcamera(cameras_);
    myslam::Camera::Ptr camera_left_;
    myslam::Camera::Ptr camera_right_;

    camera_left_ = cameras_.at(0);
    camera_right_ = cameras_.at(1);


    SE3 Twc = camera_left_->pose().inverse();
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    auto current_frame_ = myslam::Frame::CreateFrame();
    current_frame_->left_img_ = img1;
    current_frame_->right_img_ = img2;
    current_frame_->SetPose(Twc);

    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);

    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

//    cv::imshow("mask", mask);
//    cv::waitKey(0);

    std::vector<cv::KeyPoint> keypoints;
    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
    detector->detect(current_frame_->left_img_, keypoints, mask);

    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
                myslam::Feature::Ptr(new myslam::Feature(current_frame_, kp)));
        cnt_detected++;
    }

    //   LOG(INFO) << "Detect " << cnt_detected << " new features";
    cout<< "Detect " << cnt_detected << " features in left"<<endl;;

     //use LK flow to estimate points in the right image

//     std::vector<cv::Point2f> kps_left, kps_right;
//    for (auto &kp : current_frame_->features_left_) {
//        kps_left.push_back(kp->position_.pt);
//        // use same pixel in left iamge
//        kps_right.push_back(kp->position_.pt);
//    }


    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto & i : current_frame_->features_left_){
        kps_left.push_back(i->position_.pt);
        int u = i->position_.pt.x;
        int v = i->position_.pt.y;

        double depth_pixel = depth_resized.ptr<unsigned short>(v)[u]/depthScale;

        if (depth_pixel>0){
            Vec2 pixel_left(u,v);
            auto pword_project = camera_left_->pixel2world(pixel_left,camera_left_->pose(),depth_pixel); // left 2 world
            auto pixel_right_project = camera_right_->world2pixel(pword_project, camera_left_->pose()); // world 2 right
            auto u_right = pixel_right_project(0,0);
            auto v_right = pixel_right_project(1,0);
            kps_right.push_back(cv::Point2f(u_right,v_right));

        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
            current_frame_->left_img_, current_frame_->right_img_, kps_left,
            kps_right, status, error, cv::Size(21, 21), 3,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                             0.01),
            cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            myslam::Feature::Ptr feat(new myslam::Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    cout<< "good points " << cnt_detected << " in right"<<endl;
    size_t cnt_init_landmarks = 0;
    double tot_num = 0;
    double diff_tot = 0 ;
    double dist =5;
    std::unordered_map<unsigned long,Vec3> points_w;

    // triangulation
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
                camera_left_->pixel2camera(Vec2(current_frame_->features_left_[i]->position_.pt.x,current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x,current_frame_->features_right_[i]->position_.pt.y))
        };

        Vec3 pworld = Vec3::Zero();


        if (myslam::triangulation(poses, points, pworld) && pworld[2] > 0) {

            int u = current_frame_->features_left_[i]->position_.pt.x;
            int v = current_frame_->features_left_[i]->position_.pt.y;
            double d = depth.ptr<unsigned short>(v)[u];
            d=d/depthScale;
            double depth_tri = pworld(2,0);
            cout<<"tri depth "<<depth_tri<<endl;
            cout<< "pred depth "<<d<<endl;
            diff_tot = diff_tot+ abs(depth_tri-d);
            tot_num = tot_num + 1;
            cout<<"diff "<<abs(depth_tri-d)<<endl;
            //if (fabs(depth_tri-d)<dist){
                points_w.insert(make_pair(cnt_init_landmarks, pworld));
                cnt_init_landmarks++;
            //}
        }
    }

    cout<< "the totoal number before filtering "<<tot_num<<endl;
    cout<< "inliers "<< cnt_init_landmarks <<endl;

    cout << "the ave " << diff_tot / tot_num << endl;

//    //select label
//    int rows = img1.rows;
//    int cols = img1.cols;
//    Mat valid_mask = Mat::zeros(Size(cols,rows) , CV_8UC1);
//
//    val_mask(kps_left, depth,  status, valid_mask);
//    imshow("valid_mask",valid_mask);


    cout<< "Initial map created with " << cnt_init_landmarks<< " map points"<<endl;
    // Visualize( current_frame_,points_w);
    // depth to pointcloud
    double cx = camera_left_->cx_;
    double cy = camera_left_->cy_;
    double fx = camera_left_->fx_;
    double fy = camera_left_->fy_;

    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
//pointcloud.reserve(1000000);

    // cv::Mat color = img1;

    for (int v = 0; v < color.rows; v++)
        for (int u = 0; u < color.cols; u++) {
            double d = depth.ptr<unsigned short>(v)[u]; // 深度值
            if (d == 0) continue; // 为0表示没有测量到
            Eigen::Vector3d point;
            d = d /depthScale;
            point[2] = d;
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;
            Eigen::Vector3d pointWorld = Twc * point;

            Vector6d p;
            p.head<3>() = pointWorld;
            p[5] = color.data[v * color.step + u * color.channels()];   // blue
            p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
            p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
            pointcloud.push_back(p);
        }

//    for (int v = v_min; v < v_max; v++)
//        for (int u = u_min; u < u_max; u++) {
//            double d = depth.ptr<unsigned short>(v)[u]; // 深度值
//            if (d == 0) continue; // 为0表示没有测量到
//            Eigen::Vector3d point;
//            point[2] =d/depthScale ;
//            point[0] = (u - cx) * point[2] / fx;
//            point[1] = (v - cy) * point[2] / fy;
//            Eigen::Vector3d pointWorld = Twc * point;
//
//            Vector6d p;
//            p.head<3>() = pointWorld;
//            p[5] = color.data[v * color.step + u * color.channels()];   // blue
//            p[4] = color.data[v * color.step + u * color.channels() + 1]; // green
//            p[3] = color.data[v * color.step + u * color.channels() + 2]; // red
//            pointcloud.push_back(p);
//        }

    cout << "点云共有" << pointcloud.size() << "个点." << endl;
    // showPointCloud(pointcloud);
    Visualize( current_frame_,points_w,pointcloud);


    //clean code

    // check module

    // show depth

    // test CLAHE


    return 0;
}

//   cout<< left_cam->pose().matrix()<<endl;
//   cout<< right_cam->pose().matrix()<<endl;

//    imshow("image1",img1);
//    waitKey(0);
// detect key points and optical flow
// key points, using GFTT here.
//    vector<KeyPoint> kp1;
//    Ptr<GFTTDetector> detector = GFTTDetector::create(500, 0.01, 20); // maximum 500 keypoints
//    detector->detect(img1, kp1);
//    vector<Point2f> pt1, pt2;
//    for (auto &kp: kp1) pt1.push_back(kp.pt);
//    vector<uchar> status;
//    vector<float> error;
//    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
//    cv::calcOpticalFlowPyrLK(img1, img2, pt1, pt2, status, error);
//    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
//    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
//    cout << "optical flow by opencv: " << time_used.count() << endl;
//
//    Mat img2_CV;
//    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);
//    for (int i = 0; i < pt2.size(); i++) {
//        if (status[i]) {
//            cv::circle(img2_CV, pt2[i], 2, cv::Scalar(0, 250, 0), 2);
//            cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
//        }
//    }
//
//    cv::imshow("tracked by opencv", img2_CV);
//    cv::waitKey(0);
