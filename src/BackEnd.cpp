#include <opencv2/opencv.hpp>

#include "BackEnd.h"

namespace usrl_vo {
    
BackEnd::BackEnd() {

}

void BackEnd::BackEndLoop() {
    while(backend_running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        Map::KeyframesType active_kfs = map_->GetActiveKeyFrames();
        Map::LandmarksType active_landmarks = map_->GetActiveMapPoints();
        Optimize(active_kfs, active_landmarks);
    }
}

void BackEnd::Optimize(Map::KeyframesType &keyframes, Map::LandmarksType &landmarks)
{
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverCSparse<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g20::make_unique<LinearSolverType>)
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    std::map<unsigned long, VertexPose *> vertices;
    unsigned long max_kf_id = 0;
    for(auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        VertexPose *vertex_pose = new VertexePose();
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setEstimate(kf->GetPose());
        optimizer.addVertex(vertex_pose);
        if(kf->keyframe_id_ > max_kf_id) {
            max_kf_id = kf->keyframe_id_;
        }
        vertices.insert((make_pair(kf->keyframe_id_, vertex_pose)));
    }

    std::map<unsigned long, VertexXYZ *> vertices_landmarks;

    Mat33d K = camrea_left_.K_;
    SE3 left_ext = cam_left_->pose();
    SE3 left_ext = cam_right_->pose();

    int index = 1;
    double chi2_th = 5.991;
    std::map<EdgeProjection *, Feature::Ptr> edges_and_features;

    for(auto &landmark : landmarks) {
        if(landmark.second->is_outlier_) continue;
        unsigned long landmark_id = landmark.second->id_;
        auto observations = landmark.second->GetObs();
        for(auto &obs : observations) {
            if(obs.lock() == nullptr) continue;
            auto feature = obs.lock();
            if(feature->is_outlier_ || feature->frame_.lock() == nullptr) continue;

            auto frame = feature->frame_.lock();
        }
    }
}

}