#include "include/MapPoint.h"

namespace usrl_vo {

long unsigned int MapPoint::num_next_id = 0;
std::mutex MapPoint::global_mutex;

MapPoint::MapPoint(const cv::Mat &pos)
{
    pos.copyTo(world_pos_);
    id_ = num_next_id++;
}

void MapPoint::SetWorldPos(const cv::Mat &pos)
{
    std::unique_lock<std::mutex> lock2(global_mutex);
    std::unique_lock<std::mutex> lock(data_mutex_);
    pos.copyTo(world_pos_);
}

cv::Mat MapPoint::GetWorldPos()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    return world_pos_.clone();
}

void MapPoint::AddObservation(Feature* feature)
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    observations_.push_back(feature);
    observed_times_++;
}

void MapPoint::RemoveObservation(Feature* feature)
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    for(std::list<Feature*>::iterator lit = observations_.begin(), lend = observations_.end(); lit != lend; lit++)
    {
        if(*lit == feature)
        {
            lit = observations_.erase(lit);
            feature->map_point_ = static_cast<MapPoint*>(NULL);
            observed_times_--;
            break;
        }
    }
}

}