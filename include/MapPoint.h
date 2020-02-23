#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "include/Common.h"
#include "include/Feature.h"

namespace usrl_vo {

class Feature;

class MapPoint {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    long unsigned int id_;

    static long unsigned int num_next_id;
    
    bool is_outlier_ = false;

    cv::Mat world_pos_;

    static std::mutex global_mutex;

    std::mutex data_mutex_;

    int observed_times_ = 0;

    std::list<Feature*> observations_;
    

public:
    MapPoint(const cv::Mat &pos);

    cv::Mat GetWorldPos();

    void SetWorldPos(const cv::Mat &pos);
    
    void AddObservation(Feature* feature);

    void RemoveObservation(Feature* feature);
};

}

#endif