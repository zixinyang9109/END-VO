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
//DEFINE_string(config_file, "/home/yzx/Code-Reference/slambook2-master/ch13/config/Scared0103.yaml", "config file path");

DEFINE_string(config_file, "/home/yzx/CLionProjects/Dense_Tracking_and_Mapping/config/endoscope0103.yaml", "config file path");

int main(int argc, char **argv) {
//    google::InitGoogleLogging(argv[0]);
//    google::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
            new myslam::VisualOdometry(FLAGS_config_file));

    assert(vo->Init() == true);//otherwise terminate the function
    std:: cout<<"begin"<<endl;

    auto t1 = std::chrono::steady_clock::now();
    vo->Run();
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
            std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    cout << "Total cost time: " << time_used.count() << " seconds.";

    // save trajectory
    string file_name = "/home/yzx/CLionProjects/Dense_Tracking_and_Mapping/test_trajectory/trajectory02_test.txt";
    vo->SaveAllTrajectory(file_name);
    string file_name_id = "/home/yzx/CLionProjects/Dense_Tracking_and_Mapping/test_trajectory/id_trajectory02_test.txt";
    vo->SaveTrajectory(file_name_id);

    return 0 ;

}
