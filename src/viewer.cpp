//
// Created by gaoxiang on 19-5-4.
//
#include "myslam/viewer.h"
#include "myslam/feature.h"
#include "myslam/frame.h"

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>

namespace myslam {

Viewer::Viewer() {
    viewer_thread_ = std::thread(std::bind(&Viewer::ThreadLoop, this));
}

void Viewer::Close() {
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame::Ptr current_frame) {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    current_frame_ = current_frame;
}

//void Viewer::UpdateMap() {
//    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
//    assert(map_ != nullptr);
//    active_keyframes_ = map_->GetActiveKeyFrames();
//    active_landmarks_ = map_->GetActiveMapPoints();
//    map_updated_ = true;
//}

void Viewer::UpdateMap() {
    std::unique_lock<std::mutex> lck(viewer_data_mutex_);
    assert(map_ != nullptr);

    // 这样可以显示所有地图点，同时也能够看出没有回环检测，累计误差很大
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();

//    active_keyframes_ = map_->GetAllKeyFrames();
//    active_landmarks_ = map_->GetAllMapPoints();   // 改为all mappoints，显示整体地图
    map_updated_ = true;
}


void Viewer::ThreadLoop() {
    pangolin::CreateWindowAndBind("Stereo Endo", 1024, 768);
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

    while (!pangolin::ShouldQuit() && viewer_running_) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        vis_display.Activate(vis_camera);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if (current_frame_) {
            DrawFrame(current_frame_, green);
            FollowCurrentFrame(vis_camera);

            cv::Mat img = PlotFrameImage();
            cv::imshow("Endo View", img);
            cv::waitKey(10);
        }

        if (map_) {
            DrawMapPoints();
            DrawPose(); //draw trajectory
            DrawPointCloud();
        }

        pangolin::FinishFrame();
        usleep(10);
    }

    LOG(INFO) << "Stop viewer";
}



// current frame_ -> features
cv::Mat Viewer::PlotFrameImage() {
    cv::Mat img_out;
    img_out = current_frame_->color_.clone();
    //cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if (current_frame_->features_left_[i]->map_point_.lock()) {
            auto feat = current_frame_->features_left_[i];
            cv::circle(img_out, feat->position_.pt, 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return img_out;
}

void Viewer::FollowCurrentFrame(pangolin::OpenGlRenderState& vis_camera) {
    SE3 Twc = current_frame_->Pose().inverse();
    pangolin::OpenGlMatrix m(Twc.matrix());
    vis_camera.Follow(m, true);
}

// using pose to draw frame
void Viewer::DrawFrame(Frame::Ptr frame, const float* color) {
    SE3 Twc = frame->Pose().inverse();
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

// active_keyframes
void Viewer::DrawMapPoints() {
    const float red[3] = {1.0, 0, 0};
    for (auto& kf : active_keyframes_) {
        DrawFrame(kf.second, red);
    }
    glPointSize(6);
    glBegin(GL_POINTS);
    for (auto& landmark : active_landmarks_) {
        auto pos = landmark.second->Pos();
        glColor3f(red[0], red[1], red[2]);
        glVertex3d(pos[0], pos[1], pos[2]);
    }
    glEnd();
}

void Viewer::DrawPose() {
    std::unordered_map< unsigned long,  SE3> poses = map_->frame_poses_;

    if (poses.size()>2) {

        Mat44 pose;
        std::vector<unsigned long> keys;

        keys.reserve(poses.size());
        for (auto &it: poses) {
            keys.push_back(it.first);
        }
        std::sort(keys.begin(), keys.end());
//        for (auto &it: keys) {
//            pose = poses[it].inverse().matrix();
//        }

        // Draw pose
        glColor3f(0.0, 0.0, 1.0);
        glLineWidth(1.5);
        glBegin(GL_LINES);
        for (size_t i = 0; i < keys.size() - 2; i++) {
            unsigned long index_1 = keys[i];
            unsigned long index_2 = keys[i + 1];
            Mat44 p1 = poses[index_1].inverse().matrix(); // Tcw->Twc
            Mat44 p2 = poses[index_2].inverse().matrix(); // Tcw->Twc
            glVertex3d(p1(0, 3), p1(1, 3), p1(2, 3));
            glVertex3d(p2(0, 3), p2(1, 3), p2(2, 3));
        }
        glEnd();

    }




}

void Viewer::DrawPointCloud(){

    // create point cloud
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(10000); //asign space before

    // camera K
    auto fx = camera_->fx_;
    auto fy = camera_->fy_;
    auto cx = camera_->cx_;
    auto cy = camera_->cy_;

    // color map, depth, pose
    for (auto& kf : active_keyframes_){
        cv::Mat color = kf.second->color_;
        cv::Mat depth = kf.second->depth_;
        int rows = depth.rows;
        int cols = depth.cols;
        SE3 Twc = kf.second->Pose().inverse();

        //create point cloud
        for (int v = 0; v < rows; v=v+4) { //v++
            for (int u = 0; u < cols; u=u+4) { //u++
                double d = depth.ptr<double>(v)[u]; // 深度值
                if (d == 0) continue; // 为0表示没有测量到
                Eigen::Vector3d point;
                point[2] = double(d);
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
        }
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p: pointcloud) {
        glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
        glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();



}


void Viewer::DrawCurrPointCloud(){

    // create point cloud
    typedef Eigen::Matrix<double, 6, 1> Vector6d;
    std::vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);

    // camera K
    auto fx = camera_->fx_;
    auto fy = camera_->fy_;
    auto cx = camera_->cx_;
    auto cy = camera_->cy_;

    // color map, depth, pose
    auto kf = active_keyframes_.begin();

    cv::Mat color = kf->second->left_img_;
    cv::Mat depth = kf->second->depth_;
    SE3 Twc = kf->second->Pose().inverse();

    int rows = depth.rows;
    int cols = depth.cols;

    //create point cloud
    for (int v = 0; v < rows; v++) {
        for (int u = 0; u < cols; u++) {
            double d = depth.ptr<double>(v)[u]; // 深度值
            //std::cout<<"d: "<<d<<std::endl;
            if (d == 0) continue; // 为0表示没有测量到
            Eigen::Vector3d point;
            point[2] = double(d);
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
    }

    glPointSize(2);
    glBegin(GL_POINTS);
    for (auto &p: pointcloud) {
        glColor3d(p[3] / 255.0, p[4] / 255.0, p[5] / 255.0);
        glVertex3d(p[0], p[1], p[2]);
    }
    glEnd();


}

}  // namespace myslam
