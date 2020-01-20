#include <opencv2/opencv.hpp>

#include "BackEnd.h"

namespace usrl_vo {
    
BackEnd::BackEnd() {
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&BackEnd::BackEndLoop, this));
}

void BackEnd::SetMap(Map::Ptr map) {
        map_ = map
}

void BackEnd::SetCameras(Camera::Ptr left, Camera::Ptr right) {
        camera_left_ = left;
        camera_right_ = right;
    }

void BackEnd::UpdateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void BackEnd::Stop() {
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
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
        vertices.insert(make_pair(kf->keyframe_id_, vertex_pose));
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
            EdgeProjection *edge = nullptr;
            if(feature->is_on_left_image_) {
                edge = new EdgeProjection(K, left_ext);
            } else {
                edge = new EdgeProjection(K, right_ext);
            }

            if(vertices_landmarks.find(landmark_id) == vertices_landmarks.end()) {
                VertexXYZ *v = new VertexXYZ;
                v->setId(max_kf_id + landmark_id + 1);
                v->setExtimate(landmark.second->GetPos());
                v->setMarginalized(true);
                vertices_landmarks.insert(make_pair(landmark_id, v));
                optimizer.addVertex(v);
            }

            edge->setId(index);
            edge->setVertex(0, vertices.at(frame->keyframe_id_));
            edge->setVertex(1, vertices_landmarks.at(landmark_id));
            edge->setMeasurement(toVect2(feature->position_.pt));
            edge->setInformation(Mat22d::Identity());
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelat(chi2_th);
            edge->setRobustKernel(rk);
            edges_and_features.insert(make_pair(edge, feature));

            optimizer.addEdge(edge);

            index++;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    while(iteration < 5) {
        cnt_outlier = 0;
        cnt_inlier = 0;
        for(auto &ef : edges_and_features) {
            if(ef.first->chi2() > chi2_th) {
                cnt_outlier++;
            } else {
                cnt_inlier++;
            }
        }

        double inlier_ration = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if(inlier_ration > 0.5) {
            break;
        } else {
            chi2_th *= 2;
            iteration++;
        }
    }

    for(auto &ef : edges_and_features) {
        if(ef.first->chi2() > chi2_th) {
            ef.second->is_outlier_ = true;
            ef.second-map_point_.lock()->RemoveObservation(ef.second);
        } else {
            ef.second->is_outlier_ = false;
        }
    }

    LOG(INFO) << "Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier;

    for(auto &v : vertices) {
        keyframes.at(v.first)->SetPose(v.second->estimate());
    }
    for(auto &v : vertices_landmarks) {
        landmarks.at(v.first)->setPos(v.second->estimate());
    }
}

}