#include "include/Map.h"

namespace usrl_vo {
    
Map::Map():current_frame_(NULL) {
}

void Map::InsertKeyFrame(Frame* frame)
{
    current_frame_ = frame;
    if(keyframes_.find(frame->keyframe_id_) == keyframes_.end())
    {
        keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(std::make_pair(frame->keyframe_id_, frame));
    }
    else
    {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if(active_keyframes_.size() > num_active_keyframes_)
    {
        RemoveOldKeyframe();
    }
}

void Map::InsertMapPoint(MapPoint* map_point)
{
    if(landmarks_.find(map_point->id_) == landmarks_.end())
    {
        landmarks_.insert(std::make_pair(map_point->id_, map_point));
        active_landmarks_.insert(std::make_pair(map_point->id_, map_point));
    }
    else
    {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

std::unordered_map<unsigned long, MapPoint*> Map::GetAllMapPoints()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return landmarks_;
}

std::unordered_map<unsigned long, Frame*> Map::GetAllKeyFrames()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return keyframes_;
}

std::unordered_map<unsigned long, MapPoint*> Map::GetActiveMapPoints()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return active_landmarks_;
}

std::unordered_map<unsigned long, Frame*> Map::GetActiveKeyFrames()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return active_keyframes_;
}

long unsigned int Map::ActiveKeyFramesInMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return active_keyframes_.size();
}

void Map::RemoveOldKeyframe()
{
    if(!current_frame_) 
        return;
    double max_dis = 0;
    double min_dis = 9999;
    double max_kf_id = 0;
    double min_kf_id = 0;
    cv::Mat Twc = current_frame_->GetPoseInverse();

    for(std::unordered_map<unsigned long, Frame*>::iterator kf_it = active_keyframes_.begin(); kf_it != active_keyframes_.end(); kf_it++)
    {
        Frame* kf = kf_it->second;
        if(kf == current_frame_)
            continue;

        g2o::SE3Quat SE3_quat = Converter::toSE3Quat(kf->GetPose() * Twc);

        double dis = SE3_quat.log().norm();
        if(dis > max_dis)
        {
            max_dis = dis;
            max_kf_id = kf_it->first;
        }

        if(dis < min_dis)
        {
            min_dis = dis;
            min_kf_id = kf_it->first;
        }
    }

    Frame* frame_to_remove = static_cast<Frame*>(NULL);
    const double min_dis_th = 0.2;
    if(min_dis < min_dis_th)
    {
        frame_to_remove = keyframes_.at(min_kf_id);
    }
    else
    {
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    std::cout << "Remove keyframe " << frame_to_remove->keyframe_id_ << std::endl;

    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    
    for(std::vector<Feature*>::iterator f_it = frame_to_remove->features_left_.begin(), f_end = frame_to_remove->features_left_.end(); f_it != f_end; f_it++)
    {
        Feature* feature = *f_it;
        MapPoint* mp = feature->map_point_;
        if(mp)
        {
            mp->RemoveObservation(feature);
        }
    }

    for(std::vector<Feature*>::iterator f_it = frame_to_remove->features_right_.begin(), f_end = frame_to_remove->features_right_.end(); f_it != f_end; f_it++)
    {
        Feature* feature = *f_it;
        if(feature == nullptr) continue;
        MapPoint* mp = feature->map_point_;
        if(mp)
        {
            mp->RemoveObservation(feature);
        }
    }

    CleanMap();
}

void Map::CleanMap()
{
    int cnt_landmark_removed = 0;

    for(std::unordered_map<unsigned long, MapPoint*>::iterator mp_it = active_landmarks_.begin(); mp_it != active_landmarks_.end(); mp_it++)
    {
        MapPoint* landmark = mp_it->second;
        if (landmark->observed_times_ == 0)
        {
            mp_it = active_landmarks_.erase(mp_it);
            cnt_landmark_removed++;
        }
    }
    std::cout << "Removed " << cnt_landmark_removed << " active landmarks" << std::endl;
}

}