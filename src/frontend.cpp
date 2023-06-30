//
// Created by gaoxiang on 19-5-2.
//

#include <opencv2/opencv.hpp>

#include "myslam/algorithm.h"
#include "myslam/backend.h"
#include "myslam/config.h"
#include "myslam/feature.h"
#include "myslam/frontend.h"
#include "myslam/g2o_types.h"
#include "myslam/map.h"
#include "myslam/viewer.h"

namespace myslam {

Frontend::Frontend() {
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
    gftt_ =cv::GFTTDetector::create(num_features_, 0.01, 20);
    depth_diff = Config::Get<double>("depth_diff");

}

// 根据不同 status 进行不同处理
bool Frontend::AddFrame(myslam::Frame::Ptr frame) {
    current_frame_ = frame;
    std::cout<<"current is estimating "<<current_frame_->id_<<std::endl;

    switch (status_) {
        case FrontendStatus::INITING: // build initialization point
            StereoInit();
            std::cout<<"Finish Initilization"<<std::endl;
            break;
        case FrontendStatus::TRACKING_GOOD:
        case FrontendStatus::TRACKING_BAD:
            Track();
            break;
        case FrontendStatus::LOST:
            Reset();
            std::cout<<"stop at "<<current_frame_->id_<<std::endl;
            break;
    }
   // sleep(1);
    //set last frame
    last_frame_ = current_frame_;
    return true;
}

bool Frontend::Track() {
    if (last_frame_) {
        //set current pose from relative motion and last frame
        current_frame_->SetPose(relative_motion_ * last_frame_->Pose());
    }

     //return num of tracked point
    int num_track_last = TrackLastFrame();

    //return num of inliers
    tracking_inliers_ = EstimateCurrentPose();

    // Judge the status of new frame
    if (tracking_inliers_ > num_features_tracking_) {
        // tracking good
        status_ = FrontendStatus::TRACKING_GOOD;
    } else if (tracking_inliers_ > num_features_tracking_bad_) {
        // tracking bad
        status_ = FrontendStatus::TRACKING_BAD;
    } else {
        // lost
        std::cout<<"lost with trackinng_inliers"<< tracking_inliers_<<std::endl;
        status_ = FrontendStatus::LOST;
    }

    //insert frame if it is needed
    InsertKeyframe();
    // set new relative motion
    relative_motion_ = current_frame_->Pose() * last_frame_->Pose().inverse();
    //pass current frame to viewer
    if (viewer_) viewer_->AddCurrentFrame(current_frame_);

    return true;
}

bool Frontend::InsertKeyframe() {

    map_->Storepose(current_frame_);

//    if (tracking_inliers_ >= num_features_needed_for_keyframe_) {
//        // still have enough features, don't insert keyframe
//        return false;
//    }

    if (current_frame_->covered_region_ >= 60) {
        // still have enough features, don't insert keyframe
        return false;
    }

    // current frame is a new keyframe
    //From Frame class
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    LOG(INFO) << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_;

    SetObservationsForKeyFrame();
    DetectFeatures();  // detect new features

    // track in right image
    FindFeaturesInRight();
    // triangulate map points
    TriangulateNewPoints();

    keyframe_num_ = keyframe_num_ + 1;

    // update backend because we have a new keyframe
    backend_->UpdateMap();
    LOG(INFO) << "Triger backend ";
    if (viewer_) viewer_->UpdateMap();

//    if (keyframe_num_%backen_gap_){
//        // update backend because we have a new keyframe
//        backend_->UpdateMap();
//        LOG(INFO) << "Triger backend ";
//        if (viewer_) viewer_->UpdateMap();
//    }
//    else{
//        if (viewer_) viewer_->UpdateMap();
//    }

    return true;
}

void Frontend::SetObservationsForKeyFrame() {

    for (auto &feat : current_frame_->features_left_) {
        auto mp = feat->map_point_.lock();
        if (mp) mp->AddObservation(feat);
    }

}

int Frontend::TriangulateNewPoints() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    SE3 current_pose_Twc = current_frame_->Pose().inverse();
    int cnt_triangulated_pts = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.expired() &&
            current_frame_->features_right_[i] != nullptr) {
            // 左图的特征点未关联地图点且存在右图匹配点，尝试三角化
            std::vector<Vec3> points{
                camera_left_->pixel2camera(
                    Vec2(current_frame_->features_left_[i]->position_.pt.x,
                         current_frame_->features_left_[i]->position_.pt.y)),
                camera_right_->pixel2camera(
                    Vec2(current_frame_->features_right_[i]->position_.pt.x,
                         current_frame_->features_right_[i]->position_.pt.y))};
            Vec3 pworld = Vec3::Zero();

            int u = current_frame_->features_left_[i]->position_.pt.x;
            int v = current_frame_->features_left_[i]->position_.pt.y;

            if (triangulation(poses, points, pworld) && pworld[2] > 0) {
                double depth_prior = current_frame_->depth_.ptr<double>(v)[u];
                double depth_tri = pworld(2,0);
                //std::cout<< " the depth tri "<< depth_tri<<std::endl;
                //std::cout<< " the depth prior "<<depth_prior<<std::endl;
                //std::cout<< " the depth diff "<< depth_tri-depth_prior<<std::endl;
                if (fabs(depth_tri-depth_prior)<depth_diff){

                auto new_map_point = MapPoint::CreateNewMappoint();
                pworld = current_pose_Twc * pworld; // the only difference between BuildInitMap/TriangulateNewPoints
                new_map_point->SetPos(pworld);
                new_map_point->AddObservation(
                    current_frame_->features_left_[i]);
                new_map_point->AddObservation(
                    current_frame_->features_right_[i]);

                current_frame_->features_left_[i]->map_point_ = new_map_point;
                current_frame_->features_right_[i]->map_point_ = new_map_point;
                map_->InsertMapPoint(new_map_point);
                cnt_triangulated_pts++;}
            }
        }
    }
    LOG(INFO) << "new landmarks: " << cnt_triangulated_pts;
    return cnt_triangulated_pts;
}

int Frontend::EstimateCurrentPose() {
    // setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>
        LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(
            g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // vertex
    VertexPose *vertex_pose = new VertexPose();  // camera vertex_pose
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frame_->Pose());
    optimizer.addVertex(vertex_pose);

    // K
    Mat33 K = camera_left_->K();

    // edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;

    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if (mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge =
                new EdgeProjectionPoseOnly(mp->pos_, K); //position in  the world
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(
                toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Identity());
            edge->setRobustKernel(new g2o::RobustKernelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    // estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;

    for (int iteration = 0; iteration < 3; ++iteration) {
        vertex_pose->setEstimate(current_frame_->Pose());
        optimizer.initializeOptimization();
        optimizer.optimize(2);
        cnt_outlier = 0;

        // count the outliers
        for (size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if (features[i]->is_outlier_) {
                e->computeError();
            }
            if (e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            };

            if (iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
    }

    LOG(INFO) << "Outlier/Inlier in pose estimating: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    // Set pose and outlier
    current_frame_->SetPose(vertex_pose->estimate());

//    LOG(INFO) << "Current Pose = \n" << current_frame_->Pose().matrix();

    for (auto &feat : features) {
        if (feat->is_outlier_) {
            feat->map_point_.reset();
            feat->is_outlier_ = false;  // maybe we can still use it in future
        }
    }

    return features.size() - cnt_outlier;
}

int Frontend::TrackLastFrame() {

    std::vector<cv::Point2f> kps_last, kps_current;

    for (auto &kp : last_frame_->features_left_) {
//        if (kp->map_point_.lock()) {
//            // use project point
//            auto mp = kp->map_point_.lock();
//            auto px =
//                camera_left_->world2pixel(mp->pos_, current_frame_->Pose());
//            kps_last.push_back(kp->position_.pt);
//            kps_current.push_back(cv::Point2f(px[0], px[1]));
//        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
//        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        last_frame_->left_img_, current_frame_->left_img_, kps_last,
        kps_current, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;

    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the last image.";

    current_frame_->SetCoverRegion(kps_current,status);

    LOG(INFO) << "Covered Region by points " << current_frame_-> covered_region_<< " % in the last image.";

    return num_good_pts;
}

bool Frontend::StereoInit() {

    int num_features_left = DetectFeatures();
    int num_coor_features = FindFeaturesInRight();

    if (num_coor_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();

    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }

    return false;
}

int Frontend::DetectFeatures() {

    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);

    for (auto &feat : current_frame_->features_left_) {
        cv::rectangle(mask, feat->position_.pt - cv::Point2f(10, 10),
                      feat->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask);
    int cnt_detected = 0;
    for (auto &kp : keypoints) {
        current_frame_->features_left_.push_back(
            Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

int Frontend::FindFeaturesInRight() {
    // use LK flow to estimate points in the right image
    std::vector<cv::Point2f> kps_left, kps_right;
    for (auto &kp : current_frame_->features_left_) {
        kps_left.push_back(kp->position_.pt);
//        auto mp = kp->map_point_.lock();
//        if (mp) {
//            // use projected points as initial guess
//            auto px =
//                camera_right_->world2pixel(mp->pos_, current_frame_->Pose());
//            kps_right.push_back(cv::Point2f(px[0], px[1]));
//        } else {
            // use same pixel in left iamge
//        int u = kp->position_.pt.x;
//        int v = kp->position_.pt.y;
//        double depth_prior = current_frame_->depth_.ptr<double>(v)[u];
//        Vec2 pixel_left(u,v);
//        auto pword_project = camera_left_->pixel2world(pixel_left,camera_left_->pose(),depth_prior); // left 2 world
//        auto pixel_right_project = camera_right_->world2pixel(pword_project, camera_left_->pose()); // world 2 right
//        auto u_right = pixel_right_project(0,0);
//        auto v_right = pixel_right_project(1,0);
//        kps_right.push_back(cv::Point2f(u_right,v_right));

            kps_right.push_back(kp->position_.pt);
//        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(
        current_frame_->left_img_, current_frame_->right_img_, kps_left,
        kps_right, status, error, cv::Size(11, 11), 3,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                         0.01),
        cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for (size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feat(new Feature(current_frame_, kp));
            feat->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feat);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }
    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

bool Frontend::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    size_t cnt_init_landmarks = 0;
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_right_[i] == nullptr) continue;
        // create map point from triangulation
        std::vector<Vec3> points{
            camera_left_->pixel2camera(
                Vec2(current_frame_->features_left_[i]->position_.pt.x,
                     current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(
                Vec2(current_frame_->features_right_[i]->position_.pt.x,
                     current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 pworld = Vec3::Zero();

        int u = current_frame_->features_left_[i]->position_.pt.x;
        int v = current_frame_->features_left_[i]->position_.pt.y;

        if (triangulation(poses, points, pworld) && pworld[2] > 0) {
//

            double depth_prior = current_frame_->depth_.ptr<double>(v)[u];
            double depth_tri = pworld(2,0);
            //std::cout<< " the depth tri "<< depth_tri<<std::endl;
            //std::cout<< " the depth prior "<<depth_prior<<std::endl;
            //std::cout<< " the depth diff "<< depth_tri-depth_prior<<std::endl;
            if (fabs(depth_tri-depth_prior)<depth_diff){
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(pworld);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            cnt_init_landmarks++;
            map_->InsertMapPoint(new_map_point);}
        }
    }
    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

bool Frontend::Reset() {
    LOG(INFO) << "Reset is not implemented. ";
    return true;
}

}  // namespace myslam