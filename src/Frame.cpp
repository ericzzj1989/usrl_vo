#include "Frame.h"

namespace usrl_vo {

    Frame::Frame() {

    }

    Frame::Frame(long id, double time_stamp, const SE3 & pose, const cv::Mat &left, const cv::Mat &right)
        : id_(id), time_stamp_(time_stamp), pose_(pose), left_img_(left), right_img_(right) {}

    SE3 Frame::GetPose() {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        return pose_;
    }

    void Frame::SetPose(const SE3 &pose) {
        std::unique_lock<std::mutex> lck(pose_mutex_);
        pose_ = pose;
    }

    void Frame::SetKeyFrame() {
        static long keyframe_factory_id = 0;
        is_keyframe_ = true;
        keyframe_id_ = keyframe_factory_id++;
    }

    Frame::Ptr Frame::CreateFrame() {
        static long factory_id = 0;
        Frame::Ptr new_frame(new MapPoint);
        new_frame->id_ = factory_id++;
        return new_frame; 
    }
}