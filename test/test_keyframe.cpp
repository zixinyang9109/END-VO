//
// Created by yzx on 8/24/21.
//

//
// Created by Xiang on 2017/12/19.
//


#include <opencv2/opencv.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <string>

#include <chrono>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>       // std::vector

using namespace std;
using namespace cv;
using namespace cv::ximgproc;

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
//
//void slc(Mat img, Mat &labels, Mat &mask){
//    int region_size = 30;
//    int ruler = 5;//Chooses the enforcement of superpixel smoothness factor of superpixel. The bigger, the square looks like.
//    int min_element_size = 20;
//    int num_iterations = 3;
//    Mat img_blur;
//
//    cv::GaussianBlur(img,img_blur,Size(3,3),3);
//
//    //Mat converted;
//
//    //cvtColor(img_blur, converted, COLOR_BGR2HSV);
//    Ptr<cv::ximgproc::SuperpixelSLIC> slic = createSuperpixelSLIC(img,cv::ximgproc::SLIC,region_size,float(ruler));
//
//    slic->iterate(num_iterations);
//    if (min_element_size>0)
//        slic->enforceLabelConnectivity(min_element_size);
//
//    slic->getLabels(labels);
//    slic->getLabelContourMask(mask);
//
//}
//
//void val_mask( vector<Point2f> pt1, Mat img1,  vector<uchar> status, Mat& valid_mask){
//
//    // get slc segmentation
//    Mat mask,labels;
//    slc(img1,labels, mask);
//    cv::imshow("mask", mask);
//    cv::imshow("labels", labels);
//
//    double min_index, max_index;
//    minMaxIdx(labels, &min_index, &max_index);
//
//    printf("min index: %f, max index: %f\n", min_index, max_index);
//
//    int rows = img1.rows;
//    int cols = img1.cols;
//    Mat img1_CV_mask = Mat::zeros(Size(cols,rows) , CV_8UC1);
//
//
//    for (int i = 0; i < pt1.size(); i++) {
//        if (status[i]) {
//            cv::circle(img1_CV_mask, pt1[i], 2, cv::Scalar(250), 2);
//            //cv::line(img2_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
//        }
//    }
//
////    Mat val_mask = Mat::zeros(Size(cols,rows) , CV_8UC1);
//
//    for (int label_index=0; label_index < max_index; label_index++){
//        Mat sel_label = labels == label_index;
//        double valid_label = (img1_CV_mask/255.0).dot(sel_label/255);
//        //cout<< "valid_label  "<<valid_label<<endl;
//        if (valid_label>0){
//            valid_mask = valid_mask + sel_label;
//            //valid_index.push_back(label_index);
//            //cout<< "valid_index  "<<label_index<<endl;
//        }
//    }
//
//
//
//}
//SCARED_RECONSTRUCTION_rectify_black

string file_1 = "/media/yzx/Elements/SCARED_RECONSTRUCTION_rectify_black/Dataset2/keyframe_2/data/left/00000.jpg";  // first image
string file_2 = "/media/yzx/Elements/SCARED_RECONSTRUCTION_rectify_black/Dataset2/keyframe_2/data/left/00005.jpg";  // second image
//string depth_path = "/media/yzx/Elements/Result_flow/Dataset1/keyframe_2/data/depth/00010.png"; // depth

int main(int argc, char **argv) {

    // images, note they are CV_8UC1, not CV_8UC3
    Mat img1 = imread(file_1);
    Mat img2 = imread(file_2);
    //Mat depth = imread(depth_path);
    int num = 5000;//5000;


    cv::resize(img1, img1, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
    cv::resize(img2, img2, cv::Size(), 0.5, 0.5,
               cv::INTER_NEAREST);
//    cv::resize(depth, depth, cv::Size(), 0.5, 0.5,
//               cv::INTER_NEAREST);


    cv::imshow("image original", img1);

//    clahe(img1,img1);
//    clahe(img2,img2);

    Mat img1_gray,img2_gray;
    cv::imshow("image clahe", img1);
    cv::cvtColor(img1, img1_gray, CV_BGR2GRAY);

    cv::cvtColor(img2, img2_gray, CV_BGR2GRAY);
    //cv::imshow("image gray", img1);
    // cv::waitKey();

    // key points, using GFTT here.
    vector<KeyPoint> kp1;
    Ptr<GFTTDetector> detector = GFTTDetector::create(num, 0.01, 20); // maximum 500 keypoints
    detector->detect(img1_gray, kp1);

    // use opencv's flow for validation
    vector<Point2f> pt1, pt2;
    for (auto &kp: kp1) pt1.push_back(kp.pt);
    vector<uchar> status;
    vector<float> error;
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    cv::calcOpticalFlowPyrLK(img1_gray, img2_gray, pt1, pt2, status, error);
    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    auto time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);
    cout << "optical flow by opencv: " << time_used.count() << endl;

    int num_good_pts = 0;

    Mat img2_CV = img2.clone();
//    cv::cvtColor(img2, img2_CV, CV_GRAY2BGR);

    for (int i = 0; i < pt2.size(); i++) {
        if (status[i]) {
            cv::circle(img2_CV, pt2[i], 12, cv::Scalar(0, 250, 0), 2);
            //cv::line(img2_CV, pt2[i], pt1[i], cv::Scalar(0, 250, 0));
            num_good_pts++;
        }
    }

    Mat img1_CV = img1.clone();
//    cv::cvtColor(img1, img1_CV, CV_GRAY2BGR);
    for (int i = 0; i < pt1.size(); i++) {
        if (status[i]) {
            cv::circle(img1_CV, pt1[i], 12, cv::Scalar(0, 250, 0), 6);
            cv::line(img1_CV, pt1[i], pt2[i], cv::Scalar(0, 250, 0));
        }
    }
    cout<<"the number of good points "<<num_good_pts<<endl;


//    Mat img1_CV_mask;
//    cv::cvtColor(mask, img1_CV_mask, CV_GRAY2BGR);

    cv::imshow("img1", img1_CV);

    cv::imshow("img2", img2_CV);
    cout<< "good points " << num_good_pts << " in right"<<endl;

    // select label
    int rows = img1.rows;
    int cols = img1.cols;
    Mat patches = Mat::zeros(Size(cols,rows) , CV_8UC1);

    int num_patches = 8;
    int h_patch = rows/num_patches;
    int w_patch = cols/num_patches;
    double label =0;



    for (int v_i=0;v_i<num_patches;v_i++){
        for (int u_i=0;u_i<num_patches;u_i++) {

            int u = u_i * w_patch;
            int v = v_i * h_patch;

            patches(cv::Rect(u, v, w_patch, h_patch)).setTo(label);
            label++;

        }
    }

//        int value = patches.ptr<unsigned char>(v)[u];
//
//        cout<<"the pixel value "<<value<<endl;
//        cout<<"i "<<i <<endl;


    cv::imshow("patches",patches);


    double min_index, max_index;
    minMaxIdx(patches, &min_index, &max_index);

    printf("min index: %f, max index: %f\n", min_index, max_index);

    Mat point_mat = Mat::zeros(Size(cols,rows) , CV_8UC1);


    for (int i = 0; i < pt1.size(); i++) {
        if (status[i]) {
            int u  = pt1[i].x;
            int v = pt1[i].y;
            point_mat.ptr<unsigned int>(v)[u]=255;
        }
    }

    //imshow("point-mat",point_mat);
    vector<double> num_points;
    double num_val_patch = 0;


    Mat val_mask = Mat::zeros(Size(cols,rows) , CV_8UC1);

    for (int label_index=0; label_index < max_index; label_index++){
        Mat sel_label = patches == label_index;
        double valid_label = (point_mat/255.0).dot(sel_label/255);
        cout<< "valid_label  "<<valid_label<<endl;
        if (valid_label>0){
            val_mask = val_mask + sel_label;
            num_points.push_back(valid_label);
            num_val_patch++;
            //valid_index.push_back(label_index);
            //cout<< "valid_index  "<<label_index<<endl;
        }
    }

    imshow("val mask",val_mask);
    cout<< "num_val_patch  "<<num_val_patch<<endl;
    cout<<"num val "<<num_val_patch<<endl;
    cout<<"label "<<label<<endl;
    double ratio;
    ratio = num_val_patch/label;
    cout<<" the ratio is "<< ratio<<endl;


    cv::waitKey(0);

    return 0;
}


