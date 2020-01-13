#include <opencv2/opencv.hpp>

namespace usrl_vo {

FrontEnd::FrontEnd() {
    gftt_ = cv::GFTTDetector::create(Config::Get<int>("num_features"), 0.01, 20);
    num_features_init_ = Config::Get<int>("num_features_init");
    num_features_ = Config::Get<int>("num_features");
}

bool FrontEnd::AddFrame(usrl_vo::Frame::Ptr frame) {
    current_frame_ = frame;
    
    switch(status_) {
        case FrontEndStatus::INITINGL:
            StereoInit();
            break;
        case FromtEndStatus::TRACKING_GOOD:
        case FrontEndStatus::TRACKING_BAD:
            Track();
            break;
        case FrontEndStatus::LOST:
            Reset();
            break;
    }

    last_frame_ = current_frame_;
    return true;
}

bool FrontEnd::StereoInit() {
    int num_feature_left = DetectFeatures();
    int num_track_features = FindFeaturesInRight();
    if(num_track_features < num_features_init_) {
        return false;
    }

    bool build_map_success = BuildInitMap();
    if (build_map_success) {
        status_ = FrontendStatus::TRACKING_GOOD;
        if (viewer_) {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
        return true;
    }

    return false;
}

// Use goodFeatureToTrack to detect the keypoints in left image of current frame
int FrontEnd::DetectFeatures() {
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for(auto &feature : current_frame_->features_left_) {
        cv::rectangle(mask, feature->position_.pt - cv::Point2f(10, 10),
                      feature->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask); // goodFeatureToTrack
    int cnt_detected = 0;
    for(auto &kp : keypoints) {
        current_frame_->features_left_.push_back(Feature::Ptr(new Feature(current_frame_, kp)));
        cnt_detected++;
    }

    LOG(INFO) << "Detect " << cnt_detected << " new features";
    return cnt_detected;
}

// Use LK flow to estimate points in the right image
int FrontEnd::FindFeaturesInRight() {
    std::vector<cv::Point2f> kps_left, kps_right;

    for(auto &kp : current_frame_->features_left_) {
        if(kp->map_point_.lock()) {
            auto mp = kp->map_point_.lock();
            auto px = camera_left_->world2pixel(mp->pos_, current_frame_->GetPose());
            kps_left.push_back(kp.position_.pt);
            kps_right.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_left.push_back(kp.position_.pt);
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    Mat error;
    cv::calcOpticalFlowPyrLK(current_frame_->left_img_, current_frame_->right_img_, 
                             kps_left, kps_right, status, error, cv::Size(11, 11), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for(size_t i = 0; i < status.size(); ++i) {
        if (status[i]) {
            cv::KeyPoint kp(kps_right[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feature);
            num_good_pts++;
        } else {
            current_frame_->features_right_.push_back(nullptr);
        }
    }

    LOG(INFO) << "Find " << num_good_pts << " in the right image.";
    return num_good_pts;
}

bool FrontEnd::BuildInitMap() {
    std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    int cnt_init_landmarks = 0;
    for(size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        if(current_frame_->features_right_[i] == nullptr) continue; 
        std::vector<Vec3> points{
            camera_left_->pixel2camera(Vec2(current_frame_->features_left_[i]->position_.pt.x,
                                                current_frame_->features_left_[i]->position_.pt.y)),
            camera_right_->pixel2camera(Vec2(current_frame_->features_right_[i]->position_.pt.x,
                                                 current_frame_->features_right_[i]->position_.pt.y))};
        Vec3 p_world = Vec3::Zero();

        if (triangulation(poses, points, p_world) && pworld[2] > 0) {
            auto new_map_point = MapPoint::CreateNewMappoint();
            new_map_point->SetPos(p_world);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);

            current_frame_->features_left_[i]->map_point_ = new_map_point;
            current_frame_->features_right_[i]->map_point_ = new_map_point;
            map_->InsertMapPoint(new_map_point);
            cnt_init_landmarks++;
        }
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    LOG(INFO) << "Initial map created with " << cnt_init_landmarks
              << " map points";

    return true;
}

bool FrontEnd::Track() {
    if(last_frame_) {
        current_frame_->SetPose(relative_motion_ * last_frame_->GetPose())
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if(tracking_inliers_ > num_features_tracking_) {
        status_ = FrontEndStatus::TRACKING_GOOD;
    } else if(tracking_inliers_ > num_features_tracking_bad_) {
        status_ = FrontEndStatus::TRACKING_BAD;
    } else {
        status_ = FrontEndStatus::LOST;
    }

    InsertKeyFrame();
    relative_motion_ = current_frame_->GetPose() * last_frame_->GetPose().inverse();
    
    if(viewer_) viewer_->AddCurrentFrame(current_frame_);

    return true;
}

// Use LK flow to estimate points in the current frame
int FrontEnd::TrackLastFrame() {
    std::vector<cv::Point2f> kps_last, kps_current;

    for(auto &kp : last_frame_->features_left_) {
        if(kp->map_point_.lock()) {
            auto mp = kp->map_point_.lock();
            auto px = camera_left_->world2pixel(mp->pos_, current_frame_->GetPose());
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(cv::Point2f(px[0], px[1]));
        } else {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_, 
                             kps_last, kps_current, status, error, cv::Size(21, 21), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);
    
    int num_good_pts = 0;   
    for(size_t i = 0; i < status.size(); ++i) {
        if(status[i]) {
            cv::KeyPoint kp(kps_current[i], 7);
            Feature::Ptr feature(new Feature(current_frame_, kp));
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->feature_left_.push_back(feature);
            num_good_pts++;
        }
    }

    LOG(INFO) << "Find" << num_good_pts << " in the last image.";
    return num_good_pts;
}

int FrontEnd::EstimateCurrentPose() {
    //setup g2o
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    
    //vertex
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(current_frmae_->GetPose());
    optimizer.addVertex(vertex_pose);

    //K
    Mat33 K = camrea_ledt_->K();

    //edges
    int index = 1;
    std::vector<EdgeProjectionPoseOnly *> edges;
    std::vector<Feature::Ptr> features;
    for(size_t i = 0; i < current_frame_->features_left_.size(); ++i) {
        auto mp = current_frame_->features_left_[i]->map_point_.lock();
        if(mp) {
            features.push_back(current_frame_->features_left_[i]);
            EdgeProjectionPoseOnly *edge = new EdgeProjectionPoseOnly(mp->pose_, K);
            edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(toVec2(current_frame_->features_left_[i]->position_.pt));
            edge->setInformation(Eigen::Matrix2d::Indentity());
            edge->setRobustKernel(new g2o::RobustkenelHuber);
            edges.push_back(edge);
            optimizer.addEdge(edge);
            index++;
        }
    }

    //estimate the Pose the determine the outliers
    const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for(int iteration = 0; iteration < 4; ++iteration) {
        vertex_pose->setEstimate(current_frame_->GetPose());
        optimizer.initilizeOptimization();
        optimizer.optimize(10);
        cnt_outlier = 0;

        //count the outliers ??????????????????????????
        for(size_t i = 0; i < edges.size(); ++i) {
            auto e = edges[i];
            if(features[i]->is_outlier_) {
                e->computeError();
            }
            if(e->chi2() > chi2_th) {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            } else {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }

            if(iteration == 2) {
                e->setRobustKernel(nullptr);
            }
        }
        //?????????????????????????????????????????????
    }

    LOG(INFO) << "Outlier/Inlier in pose estimatin: " << cnt_outlier << "/"
              << features.size() - cnt_outlier;
    
    //Set pose
    current_frame_->SetPose(vertex_pose->estimate());

    LOG(INFO) << "Current Pose = \n" << current_frame_->GetPose().matrix();

    for(auto &feature : features) {
        if(feature->is_outlier_) {
            feature->map_point_.reset();
            feature->is_outlier_ = false;
        }
    }
    return features.size() - cnt_outlier;
}

}