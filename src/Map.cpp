#include "Map.h"

namespace usrl_vo {
    
Map::Map() {

}

void Map::InsertKeyFrame(Frame::Ptr frame) {
    current_frame_ = frame;
    if(keyframes_.find(frame->keyframe_id_) == keyframe_.end()) {
        keyframes_.insert(make_pair(frame->keyframe_id_, frame));
        active_keyframes_.insert(make_pair(frame->keyframe_id_, frame));
    } else {
        keyframes_[frame->keyframe_id_] = frame;
        active_keyframes_[frame->keyframe_id_] = frame;
    }

    if(active_keyframes_.size() > num_active_keyframes_) {
        RemoveOldKeyframe();
    }
}

void Map::InsertMapPoint(MapPoint::Ptr map_point) {
    if(landmarks_.find(map_point->id_) == landmarks_.end()) {
        landmarks_.insert(make_pair(map_point->id_, map_point));
        active_landmarks_.insert(make_pair(map_point->id_, map_point));
    } else {
        landmarks_[map_point->id_] = map_point;
        active_landmarks_[map_point->id_] = map_point;
    }
}

LandmarksType Map::GetAllMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return landmarks_;
    }

KeyframesType Map::GetAllKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return keyframes_;
    }

LandmarksType Map::GetActiveMapPoints() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_landmarks_;
    }

KeyframesType Map::GetActiveKeyFrames() {
        std::unique_lock<std::mutex> lck(data_mutex_);
        return active_keyframes_;
    }

void Map::RemoveOldKeyframe() {
    if(current_frame_ == nullptr) return;
    double max_dis = 0;
    double min_dis = 9999;
    double max_kf_id = 0;
    double min_kf_id = 0;
    auto Twc = current_frame_->GetPose().inverse();

    for(auto &kf : active_keyframes_) {
        if(kf.second == current_frame_) continue;
        auto dis = (kf.second->GetPose() * Twc).log().norm();
        if(dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if(dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first
        }
    }

    const double min_dis_th = 0.2;
    Frame::Ptr frame_to_remove = nullptr;
    if(min_dis < min_dis_th) {
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    LOG(INFO) << "Remove keyframe " << frame_to_remove->keyframe_id_;

    active_keyframes_.erase(frame_to_remove->keyframe_id_);
    for(auto feature : frame_to_remove->features_left_) {
        auto mp = feature->map_point_.lock();
        if(mp) {
            mp->RemoveObservation(feature);
        }
    }

    for(auto feature : frame_to_remove->features_right_) {
        if(feature == nullptr) continue;
        auto mp = feature->map_point_.lock();
        if(mp) {
            mp->RemoveObservation(feature);
        }
    }

    CleanMap();
}

void Map::CleanMap() {
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin(); iter != active_landmarks_.end();) {
        if (iter->second->observed_times_ == 0) {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    LOG(INFO) << "Removed " << cnt_landmark_removed << " active landmarks";
}
}