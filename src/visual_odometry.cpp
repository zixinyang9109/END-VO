//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/visual_odometry.h"
#include <chrono>
#include <iomanip>
#include "myslam/config.h"
using namespace std;
namespace myslam {

VisualOdometry::VisualOdometry(std::string &config_path)
    : config_file_path_(config_path) {}

bool VisualOdometry::Init() {
    // read from config file
    if (Config::SetParameterFile(config_file_path_) == false) {
        return false;
    }

    dataset_ =
        Dataset::Ptr(new Dataset(Config::Get<std::string>("dataset_dir")));
    CHECK_EQ(dataset_->Init(), true);

    // create components and links
    frontend_ = Frontend::Ptr(new Frontend);
    backend_ = Backend::Ptr(new Backend);
    map_ = Map::Ptr(new Map);
    viewer_ = Viewer::Ptr(new Viewer);

    frontend_->SetBackend(backend_);
    frontend_->SetMap(map_);
    frontend_->SetViewer(viewer_);
    frontend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    backend_->SetMap(map_);
    backend_->SetCameras(dataset_->GetCamera(0), dataset_->GetCamera(1));

    viewer_->SetMap(map_);
    viewer_->SetCamera(dataset_->GetCamera(0));

    return true;
}

void VisualOdometry::Run() {
    while (1) {
        LOG(INFO) << "VO is running";
      //  num_iter = num_iter+1;
//        if (num_iter>5){
//            break;
//        }
        if (Step() == false) {
            break;
        }
    }

    backend_->Stop();
    viewer_->Close();

    LOG(INFO) << "VO exit";
}

bool VisualOdometry::Step() {
    Frame::Ptr new_frame = dataset_->NextFrame();
    if (new_frame == nullptr) return false;

    auto t1 = std::chrono::steady_clock::now();
    bool success = frontend_->AddFrame(new_frame);
    auto t2 = std::chrono::steady_clock::now();
    auto time_used =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1);
    LOG(INFO) << "VO cost time: " << time_used.count() << " seconds.";

    return success;
}

void VisualOdometry::SaveTrajectory(const std::string &filename){

    std::unordered_map<unsigned long, myslam::Frame::Ptr> keyframes = map_->GetAllKeyFrames();

    //Mat44 pose;
    //int id;
    std::vector<unsigned long> keys;

    keys.reserve (keyframes.size());
    for (auto& it : keyframes) {
        keys.push_back(it.first);
    }
    std::sort (keys.begin(), keys.end());
//    for (auto& it : keys) {
//        pose = keyframes[it]->Pose().inverse().matrix() ;
//        //cout << it << ": " << pose << endl;
//        //cout<<pose(0,0) <<endl;
//    }
    // save in txt
//    ofstream outfile;
//    outfile.open("keyframe"+filename);

    ofstream outfile;
    outfile.open(filename);

    if (!outfile.is_open())//判断文件是否打开
        {
        std::cout << "Error opening file" << std::endl;
        }

//    for (auto& it : keys) {
//        pose = keyframes[it]->Pose().inverse().matrix() ;
//        // cout << it << ": " << pose << endl;
//        // cout<<pose(0,0) <<endl;
//        outfile<<std::setprecision(9)
//        <<pose(0,0)<< " " <<pose(0,1)<< " " <<pose(0,2)<< " " <<pose(0,3)<< " "
//        <<pose(1,0)<< " " <<pose(1,1)<< " " <<pose(1,2)<< " " <<pose(1,3)<<" "
//        <<pose(2,0)<< " " <<pose(2,1)<< " " <<pose(2,2)<< " " <<pose(2,3)<< endl;
//    }

    for (auto& it : keys) {
//        pose = keyframes[it]->Pose().inverse().matrix() ;
        // cout << it << ": " << pose << endl;
        // cout<<pose(0,0) <<endl;
        outfile<<std::setprecision(4)
               << keyframes[it]->id_<< endl;
    }

    outfile.close();
    cout << endl << "trajectory saved!" << endl;
}


void VisualOdometry::SaveAllTrajectory(const std::string &filename){

    std::unordered_map< unsigned long,  SE3> poses = map_->frame_poses_;

    Mat44 pose;
    std::vector<unsigned long> keys;

    keys.reserve (poses.size());
    for (auto& it : poses) {
        keys.push_back(it.first);
    }
    std::sort (keys.begin(), keys.end());
//    for (auto& it : keys) {
//        pose = poses[it].inverse().matrix() ;
//       cout << it << ": " << pose << endl;
//       cout<<pose(0,0) <<endl;
//
//    }
    // save in txt
    ofstream outfile;
    outfile.open(filename);

    if (!outfile.is_open())//判断文件是否打开
        {
        std::cout << "Error opening file" << std::endl;
        }

    for (auto& it : keys) {
        pose = poses[it].inverse().matrix() ; // Tcw->Twc
//        cout << it << ": " << pose << endl;
        // cout<<pose(0,0) <<endl;
        outfile<<std::setprecision(9)
        <<pose(0,0)<< " " <<pose(0,1)<< " " <<pose(0,2)<< " " <<pose(0,3)<< " "
        <<pose(1,0)<< " " <<pose(1,1)<< " " <<pose(1,2)<< " " <<pose(1,3)<<" "
        <<pose(2,0)<< " " <<pose(2,1)<< " " <<pose(2,2)<< " " <<pose(2,3)<< endl;
    }

    outfile.close();
    cout << endl << "trajectory saved!" << endl;
}





}  // namespace myslam
