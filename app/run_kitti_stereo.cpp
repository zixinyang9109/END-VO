//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"
#include <iostream>
#include <vector>
#include <fstream>

#include "myslam/map.h"
#include "myslam/frame.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/mappoint.h"


#include <Eigen/Dense>
#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

//DEFINE_string(config_file, "/home/yzx/Code-Reference/slambook2-master/ch13/config/default.yaml", "config file path");
DEFINE_string(config_file, "/home/yzx/CLionProjects/Dense_Tracking_and_Mapping/config/Scared0103.yaml", "config file path");

int main(int argc, char **argv) {
    //google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));

    assert(vo->Init() == true);//otherwise terminate the function
    std:: cout<<"begin"<<endl;

    vo->Run();

    // save trajectory
    string file_name = "/home/yzx/Code-Reference/slambook2-master/ch13/trajectory06.txt";
    vo->SaveAllTrajectory(file_name);


    return 0 ;
}
