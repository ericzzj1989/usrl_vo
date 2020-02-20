#include <opencv2/opencv.hpp>

#include "include/BackEnd.h"

#include "include/Converter.h"

#include<Eigen/StdVector>

namespace usrl_vo {
    
BackEnd::BackEnd(const std::string &setting_path, Map* map):
    map_(map)
{
    backend_running_.store(true);
    backend_thread_ = std::thread(std::bind(&BackEnd::BackEndLoop, this));
    
    cv::FileStorage fSettings(setting_path, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(K_);

    bf_ = fSettings["Camera.bf"];
}

// void BackEnd::SetMap(std::shared_ptr<Map> map) {
//         map_ = map;
// }

// void BackEnd::SetCameras(Camera::Ptr left, Camera::Ptr right) {
//         camera_left_ = left;
//         camera_right_ = right;
// }

void BackEnd::UpdateMap()
{
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void BackEnd::Stop()
{
    backend_running_.store(false);
    map_update_.notify_one();
    backend_thread_.join();
}

void BackEnd::BackEndLoop()
{
    while(backend_running_.load())
    {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);

        std::unordered_map<unsigned long, Frame*> active_kfs = map_->GetActiveKeyFrames();
        std::unordered_map<unsigned long, MapPoint*> active_landmarks = map_->GetActiveMapPoints();
        if(active_kfs.size() > 2)
            Optimize(active_kfs, active_landmarks);
    }
}

void BackEnd::Optimize(std::unordered_map<unsigned long, Frame*> keyframes, 
                  std::unordered_map<unsigned long, MapPoint*> landmarks)
{
    // typedef g2o::BlockSolver_6_3 BlockSolverType;
    // typedef g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType> LinearSolverType;
    // auto solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<BlockSolverType>(g20::make_unique<LinearSolverType>)
    // );

    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);

    // std::cout << "keyframes = " << keyframes.size() << std::endl;
    // std::cout << "landmarks = " << landmarks.size() << std::endl;

    std::map<unsigned long, g2o::VertexSE3Expmap*> vertices_keyframes;
    long unsigned int max_kf_id = 0;

    // cv::Mat pose_pose = keyframes.at(1)->pose_;
    // if(pose_pose.empty()) std::cout << "pose_pose pose_pose NULNULLNULLL" << std::endl;    

    for(std::unordered_map<unsigned long, Frame*>::iterator kf_it = keyframes.begin(); kf_it != keyframes.end(); kf_it++)
    {
        // std::cout << "frame id = !!!!" << kf_it->first << std::endl;
        Frame* kf = kf_it->second;
        g2o::VertexSE3Expmap* vertex_pose = new g2o::VertexSE3Expmap();
        vertex_pose->setId(kf->keyframe_id_);
        vertex_pose->setFixed(kf->keyframe_id_ == 0);
        // cv::Mat pose = kf->pose_;
        // if(pose.empty()) std::cout << "NULNULLNULLL" << std::endl;
        // std::cout << "Backend!!!!!!! = " << pose.at<float>(0,0) << std::endl;
        vertex_pose->setEstimate(Converter::toSE3Quat(kf->GetPose()));
        optimizer.addVertex(vertex_pose);
        if(kf->keyframe_id_ > max_kf_id)
        {
            max_kf_id = kf->keyframe_id_;
        }
        vertices_keyframes.insert(std::make_pair(kf->keyframe_id_, vertex_pose));
    }

    std::map<unsigned long, g2o::VertexSBAPointXYZ*> vertices_mappoints;

    // Mat33d K = camera_left_.K_;

    int index = 1;
    double thHuber = sqrt(5.99);
    std::map<EdgeStereoProjectXYZ *, Feature*> edges_and_features;

    for(std::unordered_map<unsigned long, MapPoint*>::iterator landmark_it = landmarks.begin(); landmark_it != landmarks.end(); landmark_it++)
    {
        MapPoint* map_point = landmark_it->second;
        if(map_point->is_outlier_) continue;

        long unsigned int landmark_id = map_point->id_;

        if(vertices_mappoints.find(landmark_id) == vertices_mappoints.end()) 
        {
            g2o::VertexSBAPointXYZ* vertex_mappoint = new g2o::VertexSBAPointXYZ();
            const int id = max_kf_id + landmark_id + 1;
            vertex_mappoint->setId(id);
            vertex_mappoint->setEstimate(Converter::toVector3d(map_point->GetWorldPos()));
            vertex_mappoint->setMarginalized(true);
            vertices_mappoints.insert(std::make_pair(landmark_id, vertex_mappoint));
            optimizer.addVertex(vertex_mappoint);
        }
 
        for(std::list<Feature*>::iterator f_it = map_point->observations_.begin(), f_end = map_point->observations_.end(); f_it != f_end; f_it++)
        {
            if(!(*f_it)) 
                continue;

            Feature* feature = *f_it;
            if(feature->is_outlier_ || !(feature->frame_)) 
                continue;         

            Frame* frame = feature->frame_;
            EdgeStereoProjectXYZ* edge = static_cast<EdgeStereoProjectXYZ*>(NULL);
            if(feature->is_on_left_image_)
            {
                edge = new EdgeStereoProjectXYZ();
            } else {
                edge = new EdgeStereoProjectXYZ();
            }

            Eigen::Matrix<double, 2, 1> obs;
            obs << feature->position_.pt.x, feature->position_.pt.y;
            
            edge->setId(index);
            edge->setVertex(0, vertices_keyframes.at(frame->keyframe_id_));
            edge->setVertex(1, vertices_mappoints.at(landmark_id));
            edge->setMeasurement(obs);
            edge->setInformation(Mat22d::Identity());
            // g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            // rk->setDelta(chi2_th);
            // edge->setRobustKernel(rk);

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            edge->setRobustKernel(rk);
            rk->setDelta(thHuber);

            edge->fx = K_.at<float>(0, 0);
            edge->fy = K_.at<float>(1, 1);
            edge->cx = K_.at<float>(0, 2);
            edge->cy = K_.at<float>(1, 2);
            edge->bf = bf_;

            edges_and_features.insert(std::make_pair(edge, feature));

            optimizer.addEdge(edge);

            index++;
        }
    }

    optimizer.initializeOptimization();
    optimizer.optimize(10);

    int cnt_outlier = 0, cnt_inlier = 0;
    int iteration = 0;
    double chi2_th = 7.815;
    while(iteration < 5)
    {
        cnt_outlier = 0;
        cnt_inlier = 0;
        for(std::map<EdgeStereoProjectXYZ *, Feature*>::iterator ef_it = edges_and_features.begin(), ef_end = edges_and_features.end(); ef_it != ef_end; ef_it++)
        {
            EdgeStereoProjectXYZ* edge = ef_it->first;

            if(edge->chi2() > chi2_th)
            {
                cnt_outlier++;
            }
            else
            {
                cnt_inlier++;
            }
        }

        double inlier_ration = cnt_inlier / double(cnt_inlier + cnt_outlier);
        if(inlier_ration > 0.5) 
        {
            break;
        }
        else
        {
            chi2_th *= 2;
            iteration++;
        }
    }

    std::unique_lock<std::mutex> lock(map_->map_update_mutex_);

    for(std::map<EdgeStereoProjectXYZ*, Feature*>::iterator ef_it = edges_and_features.begin(), ef_end = edges_and_features.end(); ef_it != ef_end; ef_it++)
    {
        EdgeStereoProjectXYZ* edge = ef_it->first;
        Feature* feature = ef_it->second;
        if(edge->chi2() > 7.815)
        {
            feature->is_outlier_ = true;
            feature->map_point_->RemoveObservation(feature);
        }
        else
        {
            feature->is_outlier_ = false;
        }
    }

    std::cout << "Outlier/Inlier in optimization: " << cnt_outlier << "/" << cnt_inlier << std::endl;

    for(std::map<unsigned long, g2o::VertexSE3Expmap*>::iterator f_it = vertices_keyframes.begin(), f_end = vertices_keyframes.end(); f_it != f_end; f_it++)
    {
        unsigned long id = f_it->first;
        g2o::VertexSE3Expmap* f = f_it->second;
        keyframes.at(id)->SetPose(Converter::toCvMat(f->estimate()));
    }
    
    for(std::map<unsigned long, g2o::VertexSBAPointXYZ*>::iterator l_it = vertices_mappoints.begin(), l_end = vertices_mappoints.end(); l_it != l_end; l_it++)
    {
        unsigned long id = l_it->first;
        g2o::VertexSBAPointXYZ* l = l_it->second;
        landmarks.at(id)->SetWorldPos(Converter::toCvMat(l->estimate()));
    }
}

}