#include "include/Frame.h"
#include <mutex>

namespace usrl_vo {

long unsigned int Frame::num_next_frame_id = 0;
long unsigned int Frame::num_next_keyframe_id = 0;

Frame::Frame(const Frame &frame):
    id_(frame.id_), keyframe_id_(frame.keyframe_id_), is_keyframe_(frame.is_keyframe_),
    left_img_(frame.left_img_.clone()), right_img_(frame.right_img_.clone()), time_stamp_(frame.time_stamp_)
{
    if(!frame.pose_.empty())
        SetPose(frame.pose_);
}

Frame::Frame(const cv::Mat &left, const cv::Mat &right, const double &time_stamp)
    :left_img_(left.clone()), right_img_(right.clone()), time_stamp_(time_stamp) 
{
    id_ = num_next_frame_id++;

    is_keyframe_ = false;
}

void Frame::SetPose(const cv::Mat &pose)
{
    std::unique_lock<std::mutex> lock(pose_mutex_);
    pose.copyTo(pose_);
    Rcw_ = pose_.rowRange(0,3).colRange(0,3);
    tcw_ = pose_.rowRange(0,3).col(3);
    Rwc_ = Rcw_.t();
    twc_ = -Rwc_ * tcw_;

    pose_inv_ = cv::Mat::eye(4, 4, pose_.type());
    Rwc_.copyTo(pose_inv_.rowRange(0,3).colRange(0,3));
    twc_.copyTo(pose_inv_.rowRange(0,3).col(3));
}

cv::Mat Frame::GetPose()
{
    std::unique_lock<std::mutex> lock(pose_mutex_);
    return pose_.clone();
}

cv::Mat Frame::GetPoseInverse()
{
    std::unique_lock<std::mutex> lock(pose_mutex_);
    return pose_inv_.clone();
}

void Frame::SetKeyFrame()
{
    is_keyframe_ = true;
    keyframe_id_ = num_next_keyframe_id++;
}

// Frame* Frame::CreateFrame() {
//     static long factory_id = 0;
//     Frame* new_frame(new Frame);
//     new_frame->id_ = factory_id++;
//     return new_frame; 
// }

}