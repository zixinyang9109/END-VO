#include "myslam/dataset.h"
#include "myslam/frame.h"

#include <boost/format.hpp>
#include <fstream>
#include <opencv2/opencv.hpp>
using namespace std;

namespace myslam {

Dataset::Dataset(const std::string& dataset_path)
    : dataset_path_(dataset_path) {}

//bool Dataset::Init() {
//    // work cameras_
//
//    // read camera intrinsics and extrinsics
//    ifstream fin(dataset_path_ + "/calib.txt");
//    if (!fin) {
//        LOG(ERROR) << "cannot find " << dataset_path_ << "/calib.txt!";
//        return false;
//    }
//
//    for (int i = 0; i < 2; ++i) {
//        char camera_name[3];
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
//            projection_data[4], projection_data[5], projection_data[6],
//            projection_data[8], projection_data[9], projection_data[10];
//
//        Vec3 t;
//        t << projection_data[3], projection_data[7], projection_data[11];
//        t = K.inverse() * t;
//        K = K * 0.5;
//
//        Camera::Ptr new_camera(new Camera(K(0, 0), K(1, 1), K(0, 2), K(1, 2),
//                                          t.norm(), SE3(SO3(), t)));
//
//        cameras_.push_back(new_camera);
//        LOG(INFO) << "Camera " << i << " intrinsics: " << new_camera->K();
//        LOG(INFO) << "Camera " << i << " extrinsics: " << t;
//
//    }
//
//    fin.close();
//
//    current_image_index_ = 0;
//
//    return true;
//}


bool Dataset::Init() {
    // read camera intrinsics and extrinsics
    depth_path_ = Config::Get<std::string>("depth_dir");
    depth_scale_ = Config::Get<double> ("depth_scale");
    resize_scale_ = Config::Get<double> ("resize_scale");
    gray_ = Config::Get<int> ("gray");

    auto v_min= myslam::Config::Get<int> ("v_min");
    auto v_max = v_min + myslam::Config::Get<int> ("v_h");

    auto u_min = myslam::Config::Get<int> ("u_min");
    auto u_max = u_min + myslam::Config::Get<int> ("u_w");

    v_min_ = v_min * resize_scale_+5;
    v_max_ = v_max * resize_scale_-5;
    u_min_ = u_min * resize_scale_+5;
    u_max_ = u_max * resize_scale_-5;

    auto fx1= Config::Get<double> ("camera1.fx");
    auto fy1= Config::Get<double> ("camera1.fy");
    auto cx1= Config::Get<double> ("camera1.cx");
    auto cy1= Config::Get<double> ("camera1.cy");

    auto fx2= Config::Get<double> ("camera2.fx");
    auto fy2= Config::Get<double> ("camera2.fy");
    auto cx2= Config::Get<double> ("camera2.cx");
    auto cy2= Config::Get<double> ("camera2.cy");

    auto tx= Config::Get<double> ("tx");
    auto ty= Config::Get<double> ("ty");
    auto tz= Config::Get<double> ("tz");

    Vec3 t1;
    Vec3 t2;

    t1 << 0, 0, 0;
    t2 << tx, ty, tz;

    Mat33 K1;
    K1 <<fx1, 0, cx1, 0, fy1, cy1, 0, 0, 1;

    Mat33 K2;
    K2 <<fx2, 0, cx2, 0, fy2, cy2, 0, 0, 1;

    //t2 = K.inverse() * t2;
    K1 = K1 * resize_scale_;
    K2 = K2 * resize_scale_;

    myslam::Camera::Ptr new_camera0(new Camera(K1(0, 0), K1(1, 1), K1(0, 2), K1(1, 2),
                                          t1.norm(), SE3(SO3(), t1)));

    //new_camera0->SetRoi(v_min,v_max,u_min,u_max);

    LOG(INFO) << "Camera " << 0 << " intrinsics: " << new_camera0->K();
    LOG(INFO) << "Camera " << 0 << " extrinsics: " << t1;

    cameras_.push_back(new_camera0);

    myslam::Camera::Ptr new_camera1(new Camera(K2(0, 0), K2(1, 1), K2(0, 2), K2(1, 2),
                                               t2.norm(), SE3(SO3(), t2)));

    LOG(INFO) << "Camera " << 1 << " intrinsics: " << new_camera1->K();
    LOG(INFO) << "Camera " << 1 << " extrinsics: " << t2;

    cameras_.push_back(new_camera1);

    current_image_index_ = 0;

    return true;
}


void Dataset::clahe(cv::Mat bgr_image,cv::Mat &image_clahe)
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

Frame::Ptr Dataset::NextFrame() {
//boost::format fmt("%s/image_%d/%06d.png");
//    boost::format fmt1("%s/image_0/%05d.jpg");
//    boost::format fmt2("%s/image_1/%05d.jpg");
    cv::Mat image_left, image_right, depth_unscale;
    cv::Mat color; string color_path;
    string depth_path,left_path,right_path;

    if (gray_==0){

        cout<<"Use gray"<<endl;
        boost::format fmt1("%s/image_0/%05d.jpg");
        boost::format fmt2("%s/image_1/%05d.jpg");
        boost::format fmt3("%s/%05d.png");
        boost::format fmt4("%s/left/%05d.jpg");

        left_path = (fmt1 % dataset_path_  % current_image_index_).str();
        right_path = (fmt2 % dataset_path_ % current_image_index_).str();
        depth_path = (fmt3 % depth_path_ % current_image_index_).str();
        color_path = (fmt4 % dataset_path_  % current_image_index_).str();
        // read images
        image_left =cv::imread(left_path,cv::IMREAD_GRAYSCALE);
        image_right =cv::imread(right_path,cv::IMREAD_GRAYSCALE);
        color = cv::imread(color_path);

        if (image_left.data == nullptr || image_right.data == nullptr) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }

    }else{
        cout<<"Use color"<<endl;
        boost::format fmt1("%s/left/%05d.jpg");
        boost::format fmt2("%s/right/%05d.jpg");
        boost::format fmt3("%s/%05d.png");

        left_path = (fmt1 % dataset_path_  % current_image_index_).str();
        right_path = (fmt2 % dataset_path_ % current_image_index_).str();
        depth_path = (fmt3 % depth_path_ % current_image_index_).str();
        image_left = cv::imread(left_path);
        image_right = cv::imread(right_path);


        if (image_left.data == nullptr || image_right.data == nullptr) {
            LOG(WARNING) << "cannot find images at index " << current_image_index_;
            return nullptr;
        }

        clahe(image_left,image_left);
        clahe(image_right,image_right);
        cv::cvtColor(image_left,image_left, CV_BGR2GRAY);
        cv::cvtColor(image_right,image_right, CV_BGR2GRAY);
        color =image_left;

    }
//    cout<<" left image name "<< left_path<<endl;
//    cout<<" right image name "<<right_path<<endl;

    // read images
//    image_left =
//            cv::imread(left_path,
//                       cv::IMREAD_GRAYSCALE);
//    image_right =
//            cv::imread(right_path,
//                       cv::IMREAD_GRAYSCALE);



    depth_unscale = cv::imread(depth_path);
    cv::resize(depth_unscale, depth_unscale, cv::Size(), resize_scale_, resize_scale_,
               cv::INTER_NEAREST);

    int rows = depth_unscale.rows;
    int cols = depth_unscale.cols;
    Mat depth = Mat::zeros(cv::Size(cols,rows) , CV_64FC1);

    for (int v = v_min_; v < v_max_; v++)
        for (int u = u_min_; u < u_max_; u++) {
            double d = depth_unscale.ptr<unsigned short>(v)[u]; // 深度值
//            cout<<d<<endl;
            d = d/depth_scale_;
            depth.ptr<double> (v)[u]= d;
            //cout<<"check the depth value "<<d<<endl;

        }


//    depth = depth/depth_scale_;



    //cv::Mat image_left_resized, image_right_resized;
    cv::resize(image_left, image_left, cv::Size(), resize_scale_, resize_scale_,
               cv::INTER_NEAREST);
    cv::resize(image_right, image_right, cv::Size(), resize_scale_, resize_scale_,
               cv::INTER_NEAREST);


    //cv::cvtColor(color,color, CV_BGR2RGB);
    cv::resize(color, color, cv::Size(), resize_scale_, resize_scale_,
               cv::INTER_NEAREST);


    auto new_frame = Frame::CreateFrame();
    new_frame->left_img_ = image_left;
    new_frame->right_img_ = image_right;
    new_frame->depth_= depth;
    new_frame->color_= color;
    current_image_index_++;

    return new_frame;
}



//Frame::Ptr Dataset::NextFrame() {
////boost::format fmt("%s/image_%d/%06d.png");
//    boost::format fmt("%s/image_%d/%05d.jpg");
//
//    cv::Mat image_left, image_right;
//    // read images
//    image_left =
//            cv::imread((fmt % dataset_path_ % 0 % current_image_index_).str(),
//                       cv::IMREAD_GRAYSCALE);
//    image_right =
//            cv::imread((fmt % dataset_path_ % 1 % current_image_index_).str(),
//                       cv::IMREAD_GRAYSCALE);
//
//    if (image_left.data == nullptr || image_right.data == nullptr) {
//        LOG(WARNING) << "cannot find images at index " << current_image_index_;
//        return nullptr;
//    }
//
//    cv::Mat image_left_resized, image_right_resized;
//    cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
//               cv::INTER_NEAREST);
//    cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
//               cv::INTER_NEAREST);
//
//    auto new_frame = Frame::CreateFrame();
//    new_frame->left_img_ = image_left_resized;
//    new_frame->right_img_ = image_right_resized;
//    current_image_index_++;
//    return new_frame;
//}

}  // namespace myslam