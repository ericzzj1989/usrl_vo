#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "include/Common.h"
#include "include/Feature.h"

namespace usrl_vo {

class Feature;

class MapPoint {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    typedef std::shared_ptr<MapPoint> Ptr;
    long unsigned int id_ = 0;
    bool is_outlier_ = false;
    cv::Mat world_pos_;
    static std::mutex global_mutex;
    std::mutex data_mutex_;
    int observed_times_ = 0;
    std::list<Feature*> observations_;
    static long unsigned int num_next_id;

public:
    MapPoint(const cv::Mat &pos);

    // MapPoint(long id, Vec3d position);

    cv::Mat GetWorldPos();

    void SetWorldPos(const cv::Mat &pos);
    
    void AddObservation(Feature* feature);

    void RemoveObservation(Feature* feature);

    // std::list<std::weak_ptr<Feature>> GetObs();

    // MapPoint* static CreateNewMapPoint();

};

}

#endif