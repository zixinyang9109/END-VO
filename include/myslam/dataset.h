#ifndef MYSLAM_DATASET_H
#define MYSLAM_DATASET_H
#include "myslam/camera.h"
#include "myslam/common_include.h"
#include "myslam/frame.h"
#include "myslam/config.h"

namespace myslam {

/**
 * 数据集读取
 * 构造时传入配置文件路径，配置文件的dataset_dir为数据集路径
 * Init之后可获得相机和下一帧图像
 */
class Dataset {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<Dataset> Ptr;
    Dataset(const std::string& dataset_path);

    /// 初始化，返回是否成功
    bool Init();

//    ///clahe
    void clahe(cv::Mat bgr_image,cv::Mat &image_clahe);


    /// create and return the next frame containing the stereo images
    Frame::Ptr NextFrame();

    /// get camera by id
    Camera::Ptr GetCamera(int camera_id) const {
        return cameras_.at(camera_id);
    }

   private:
    std::string dataset_path_;
    std::string depth_path_;
    double depth_scale_;
    double resize_scale_;
    int gray_;
    int current_image_index_ = 0;

    int v_min_,v_max_,u_min_,u_max_;

    std::vector<Camera::Ptr> cameras_;
};
}  // namespace myslam

#endif