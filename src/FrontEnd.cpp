#include <opencv2/opencv.hpp>

#include "include/FrontEnd.h"

namespace usrl_vo {

FrontEnd::FrontEnd(const std::string &setting_path, Map* map, BackEnd* backend):
    status_(NO_IMAGES), map_(map), backend_(backend), viewer_(NULL)
   // current_frame_(NULL), new_frame_(NULL), last_frame_(NULL)
{
    cv::FileStorage fSettings(setting_path, cv::FileStorage::READ);

    float fx = fSettings["Camera.fx"];
    float fy = fSettings["Camera.fy"];
    float cx = fSettings["Camera.cx"];
    float cy = fSettings["Camera.cy"];

    cv::Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = fx;
    K.at<float>(1,1) = fy;
    K.at<float>(0,2) = cx;
    K.at<float>(1,2) = cy;
    K.copyTo(K_);

    bf_ = fSettings["Camera.bf"];

    cv::Mat R_rl = cv::Mat::eye(3, 3, CV_32F);
    R_rl.copyTo(R_rl_);
    
    cv::Mat t_rl = cv::Mat(3, 1, CV_32F);
    t_rl.at<float>(0) = -bf_ / K_.at<float>(0,0);
    t_rl.at<float>(1) = 0;
    t_rl.at<float>(2) = 0;
    t_rl.copyTo(t_rl_);

    num_features_ = fSettings["ORBextractor.nFeatures"];
    gftt_ = cv::GFTTDetector::create(num_features_, 0.01, 10);
    num_features_init_ = fSettings["nFeatures"];
}

void FrontEnd::SetViewer(Viewer* viewer) {
        viewer_ = viewer;
}

cv::Mat FrontEnd::ProcessImageStereo(const cv::Mat &im_rect_left, const cv::Mat &im_rect_right, const double &time_stamp) {
    cv::Mat im_gray_left = im_rect_left;
    cv::Mat im_gray_right = im_rect_right;

    if(3 == im_gray_left.channels()) {
        cvtColor(im_gray_left, im_gray_left, CV_RGB2GRAY);
        cvtColor(im_gray_right, im_gray_right, CV_RGB2GRAY);
    }
    else if(4 == im_gray_left.channels()) {
        cvtColor(im_gray_left, im_gray_left, CV_RGBA2GRAY);
        cvtColor(im_gray_right, im_gray_right, CV_RGBA2GRAY);
    }
    
    current_frame_ = new Frame(im_gray_left, im_gray_right, time_stamp);
    
    Track();

    return current_frame_->pose_.clone();
}

void FrontEnd::Track()
{
    if(NO_IMAGES == status_)
    {
        status_ = NOT_INITIALIZED;
    }

    std::unique_lock<std::mutex> lock(map_->map_update_mutex_);

    if(NOT_INITIALIZED == status_)
    {
        StereoInit();

        if(TRACKING_GOOD != status_)
            return;
    }
    else
    {
        bool bOK;

        if(TRACKING_GOOD == status_|| TRACKING_BAD == status_)
        {
            bOK = TrackLoop();
        }

        if(!bOK)
            status_ = LOST;

        if(bOK)
        {
            CreateNewKeyFrame();
        }

        if(LOST == status_)
        {
            Reset();
            return;
        }

        last_frame_ = current_frame_;
    }

    if(!current_frame_->pose_.empty())
    {
        cv::Mat T_cl = current_frame_->pose_ * last_frame_->GetPoseInverse();
        relative_poses_.push_back(T_cl);
    }
    else
    {
        relative_poses_.push_back(relative_poses_.back());
    }
    
}

void FrontEnd::StereoInit()
{
    current_frame_->SetPose(cv::Mat::eye(4,4,CV_32F));

    int num_feature_left = DetectFeatures();
    int num_track_features = FindFeaturesInRight();
    if(num_feature_left < num_features_init_ || num_track_features < num_features_init_)
    {
        std::cout << "Wrong initialization, reseting..." << std::endl;
        Reset();
        return;
    }

    bool build_map_success = BuildInitMap();
    if(build_map_success)
    {
        // std::cout << "MAP BUILD Success !!!!!!" << std::endl;
        status_ = TRACKING_GOOD;
        if (viewer_)
        {
            viewer_->AddCurrentFrame(current_frame_);
            viewer_->UpdateMap();
        }
    }

    last_frame_ = current_frame_;
}

// Use goodFeatureToTrack to detect the keypoints in left image of current frame
int FrontEnd::DetectFeatures()
{
    cv::Mat mask(current_frame_->left_img_.size(), CV_8UC1, 255);
    for(std::vector<Feature*>::iterator f_it = current_frame_->features_left_.begin(), f_end = current_frame_->features_left_.end(); f_it != f_end; f_it++)
    {
        Feature* feature = *f_it;
        cv::rectangle(mask, feature->position_.pt - cv::Point2f(10, 10),
                      feature->position_.pt + cv::Point2f(10, 10), 0, CV_FILLED);
    }

    std::vector<cv::KeyPoint> keypoints;
    gftt_->detect(current_frame_->left_img_, keypoints, mask); // goodFeatureToTrack
    int cnt_detected = 0;
    for(std::vector<cv::KeyPoint>::iterator kp_it = keypoints.begin(), kp_end = keypoints.end(); kp_it != kp_end; kp_it++)
    {
        Feature* feature = new Feature(current_frame_, *kp_it);
        current_frame_->features_left_.push_back(feature);
        cnt_detected++;
    }

    std::cout << "Detect " << cnt_detected << " new features" << std::endl;
    return cnt_detected;
}

// Use LK flow to estimate points in the right image
int FrontEnd::FindFeaturesInRight()
{
    std::vector<cv::Point2f> kps_left, kps_right;

    for(std::vector<Feature*>::iterator f_it = current_frame_->features_left_.begin(), f_end = current_frame_->features_left_.end(); f_it != f_end; f_it++)
    {
        Feature* kp = *f_it;
        if(kp->map_point_)
        {
            MapPoint* mp = kp->map_point_;
            cv::Mat w_pos = mp->world_pos_;
            cv::Point2f px = WorldToPixelinRight(mp->world_pos_, current_frame_->pose_, K_, bf_);
            kps_left.push_back(kp->position_.pt);
            kps_right.push_back(px);
        } else {
            kps_left.push_back(kp->position_.pt);
            kps_right.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;
    cv::calcOpticalFlowPyrLK(current_frame_->left_img_, current_frame_->right_img_, 
                             kps_left, kps_right, status, error, cv::Size(11, 11), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);

    int num_good_pts = 0;
    for(size_t i = 0; i < status.size(); ++i)
    {
        if (status[i])
        {
            cv::KeyPoint kp(kps_right[i], 1.f);
            Feature* feature = new Feature(current_frame_, kp);
            feature->is_on_left_image_ = false;
            current_frame_->features_right_.push_back(feature);
            num_good_pts++;
        }
        else
        {
            current_frame_->features_right_.push_back(static_cast<Feature*>(NULL));
        }
    }

    std::cout << "Find " << num_good_pts << " in the right image." << std::endl;
    return num_good_pts;
}

bool FrontEnd::BuildInitMap()
{
    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    K_.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat P2(3,4,CV_32F);
    R_rl_.copyTo(P2.rowRange(0,3).colRange(0,3));
    t_rl_.copyTo(P2.rowRange(0,3).col(3));
    P2 = K_ * P2;

    // std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    int cnt_init_landmarks = 0;
    for(size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        if(!current_frame_->features_right_[i]) 
            continue;

        const cv::KeyPoint &kp_l = current_frame_->features_left_[i]->position_;
        const cv::KeyPoint &kp_r = current_frame_->features_right_[i]->position_;
        cv::Mat p_3d;

        Triangulate(kp_l, kp_r, P1, P2, p_3d);

        if(p_3d.at<float>(2)<=0)
            continue;

        MapPoint* new_map_point = new MapPoint(p_3d);
        new_map_point->AddObservation(current_frame_->features_left_[i]);
        new_map_point->AddObservation(current_frame_->features_right_[i]);

        current_frame_->features_left_[i]->map_point_ = new_map_point;
        current_frame_->features_right_[i]->map_point_ = new_map_point;
        map_->InsertMapPoint(new_map_point);
        cnt_init_landmarks++;
    }

    // Frame* ini_frame = new Frame(current_frame_);
    // reference_frame_ = ini_frame;

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);
    backend_->UpdateMap();

    std::cout << "Initial map created with " << cnt_init_landmarks
              << " map points" << std::endl;;

    return true;
}

bool FrontEnd::TrackLoop()
{
    cv::Mat T_cl = relative_poses_.back();

    if(!last_frame_->pose_.empty())
    {
        current_frame_->SetPose(T_cl * last_frame_->GetPose());
    }

    int num_track_last = TrackLastFrame();
    tracking_inliers_ = EstimateCurrentPose();

    if(num_track_last > num_features_tracking_good_ && tracking_inliers_ > num_features_tracking_good_)
    {
        status_ = TRACKING_GOOD;

        return true;
    }
    else if(tracking_inliers_ > num_features_tracking_bad_)
    {
        status_ = TRACKING_BAD;

        return true;
    }
    else
    {
        return false;
    }
    
    if(viewer_) viewer_->AddCurrentFrame(current_frame_);

    return true;
}

// Use LK flow to estimate points in the current frame
int FrontEnd::TrackLastFrame()
{
    std::vector<cv::Point2f> kps_last, kps_current;

    for(std::vector<Feature*>::iterator f_it = last_frame_->features_left_.begin(), f_end = last_frame_->features_left_.end(); f_it != f_end; f_it++)
    {
        Feature* kp = *f_it;
        if(kp->map_point_)
        {
            MapPoint* mp = kp->map_point_;
            cv::Point2f px = WorldToPixelinLeft(mp->world_pos_, current_frame_->pose_, K_);
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(px);
        }
        else
        {
            kps_last.push_back(kp->position_.pt);
            kps_current.push_back(kp->position_.pt);
        }
    }

    std::vector<uchar> status;
    cv::Mat error;

    std::cout << "kps_last size() ====" << kps_last.size() << std::endl;
    std::cout << "kps_current size() ====" << kps_current.size() << std::endl;

    cv::calcOpticalFlowPyrLK(last_frame_->left_img_, current_frame_->left_img_, 
                             kps_last, kps_current, status, error, cv::Size(21, 21), 3,
                             cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
                             cv::OPTFLOW_USE_INITIAL_FLOW);
    
    int num_good_pts = 0;   
    for(size_t i = 0; i < status.size(); ++i)
    {
        if(status[i])
        {
            cv::KeyPoint kp(kps_current[i], 1.f);
            Feature* feature = new Feature(current_frame_, kp);
            feature->map_point_ = last_frame_->features_left_[i]->map_point_;
            current_frame_->features_left_.push_back(feature);
            num_good_pts++;
        }
    }

    std::cout << "Find " << num_good_pts << " in the last image." << std::endl;
    return num_good_pts;
}

int FrontEnd::EstimateCurrentPose()
{
    //setup g2o
    // typedef g2o::BlockSolver_6_3 BlockSolverType;
    // typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    // auto solver = new g2o::OptimizationAlgorithmLevenberg(
    //     g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>));
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;

    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolver_6_3::PoseMatrixType>();

    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);

    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    
    //vertex
    g2o::VertexSE3Expmap *vertex_pose = new g2o::VertexSE3Expmap();
    vertex_pose->setId(0);
    vertex_pose->setEstimate(Converter::toSE3Quat(current_frame_->pose_));
    vertex_pose->setFixed(false);
    optimizer.addVertex(vertex_pose);

    //edges
    // int index = 1;
    const double delta = sqrt(5.991);
    std::vector<g2o::EdgeSE3ProjectXYZOnlyPose*> edges;
    std::vector<Feature*> features;
    for(size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        // std::cout << "current_frame_->features_left_[i] = ???????" << 
        // current_frame_->features_left_[i]->position_.pt.x << std::endl;
        MapPoint* mp = current_frame_->features_left_[i]->map_point_;
        // std::cout << "map point " << mp->pos_[0] << " " << mp->pos_[1] << " " << mp->pos_[2] << std::endl;
        if(mp)
        {
            Eigen::Matrix<double,2,1> obs;
            obs << current_frame_->features_left_[i]->position_.pt.x, current_frame_->features_left_[i]->position_.pt.y;
            features.push_back(current_frame_->features_left_[i]);
            g2o::EdgeSE3ProjectXYZOnlyPose* edge = new g2o::EdgeSE3ProjectXYZOnlyPose();
            // edge->setId(index);
            edge->setVertex(0, vertex_pose);
            edge->setMeasurement(obs);
            edge->setInformation(Eigen::Matrix2d::Identity());

            g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
            edge->setRobustKernel(rk);
            rk->setDelta(delta);
            // edge->setRobustKernel(new g2o::RobustKernelHuber);

            edge->fx = K_.at<float>(0, 0);
            edge->fy = K_.at<float>(1, 1);
            edge->cx = K_.at<float>(0, 2);
            edge->cy = K_.at<float>(1, 2);
            cv::Mat w_pos = mp->GetWorldPos();
            edge->Xw[0] = w_pos.at<float>(0);
            edge->Xw[1] = w_pos.at<float>(1);
            edge->Xw[2] = w_pos.at<float>(2);

            edges.push_back(edge);
            optimizer.addEdge(edge);
            // index++;

            // std::cout << "Current features????? = \n" << features.size() << std::endl;
        }
    }

    const float chi2Stereo[4] = {7.815, 7.815, 7.815, 7.815};
    const int its[4] = {10, 10, 10, 10};
    
    //estimate the Pose and determine the outliers
    // const double chi2_th = 5.991;
    int cnt_outlier = 0;
    for(std::size_t iteration = 0; iteration < 4; iteration++)
    {
        vertex_pose->setEstimate(Converter::toSE3Quat(current_frame_->pose_));
        optimizer.initializeOptimization(0);
        optimizer.optimize(its[iteration]);
        cnt_outlier = 0;

        std::cout << "edge size ======== " << edges.size() << std::endl;
        for(std::size_t i = 0, i_end = edges.size(); i < i_end; i++)
        {
            g2o::EdgeSE3ProjectXYZOnlyPose* e = edges[i];

            if(features[i]->is_outlier_)
            {
                e->computeError();
            }

            const float chi2 = e->chi2();

            if(chi2 > chi2Stereo[iteration])
            {
                features[i]->is_outlier_ = true;
                e->setLevel(1);
                cnt_outlier++;
            }
            else
            {
                features[i]->is_outlier_ = false;
                e->setLevel(0);
            }

            if(iteration == 2)
            {
                e->setRobustKernel(0);
            }
        }
        if(optimizer.edges().size()<10)
            break;
    } 

    std::cout << "Outlier/Inlier in pose estimatin: " << cnt_outlier << "/"
              << features.size() - cnt_outlier << std::endl;
    
    //Set pose
    g2o::VertexSE3Expmap* vertex_pose_recov = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(0));
    g2o::SE3Quat SE3quat_recov = vertex_pose_recov->estimate();
    cv::Mat pose = Converter::toCvMat(SE3quat_recov);
    current_frame_->SetPose(pose);

    std::cout << "Current Pose = \n" << current_frame_->pose_ << std::endl;

    for(std::vector<Feature*>::iterator f_it = features.begin(), f_end = features.end(); f_it != f_end; f_it++)
    {
        Feature* feature = *f_it;
        if(feature->is_outlier_) {
            feature->map_point_ = static_cast<MapPoint*>(NULL);
            feature->is_outlier_ = false;
        }
    }
    return features.size() - cnt_outlier;
}

bool FrontEnd::CreateNewKeyFrame()
{
    if(tracking_inliers_ >= num_features_needed_for_keyframe_)
    {
        return false;
    }

    current_frame_->SetKeyFrame();
    map_->InsertKeyFrame(current_frame_);

    std::cout << "Set frame " << current_frame_->id_ << " as keyframe "
              << current_frame_->keyframe_id_ << std::endl;

    SetObservationsForKeyFrame();
    DetectFeatures();

    FindFeaturesInRight();
    TriangulateNewPoints();
    backend_->UpdateMap();

    if(viewer_) viewer_->UpdateMap();

    return true;
}

void FrontEnd::SetObservationsForKeyFrame()
{
    
    for(std::vector<Feature*>::iterator f_it = current_frame_->features_left_.begin(), f_end = current_frame_->features_left_.end(); f_it != f_end; f_it++)
    {
        Feature* feature = *f_it;
        MapPoint* mp = feature->map_point_;
        if(mp) mp->AddObservation(feature);
    }
}

int FrontEnd::TriangulateNewPoints()
{
    cv::Mat P1(3, 4, CV_32F, cv::Scalar(0));
    K_.copyTo(P1.rowRange(0,3).colRange(0,3));

    cv::Mat P2(3,4,CV_32F);
    R_rl_.copyTo(P2.rowRange(0,3).colRange(0,3));
    t_rl_.copyTo(P2.rowRange(0,3).col(3));
    P2 = K_ * P2;

    // std::vector<SE3> poses{camera_left_->pose(), camera_right_->pose()};
    cv::Mat T_wc = current_frame_->GetPoseInverse(); // T_wc:c->w
    cv::Mat R_wc = T_wc.rowRange(0,3).colRange(0,3);
    cv::Mat t_wc = T_wc.rowRange(0,3).col(3);

    int cnt_triangulated_pts = 0;
    for(size_t i = 0; i < current_frame_->features_left_.size(); i++)
    {
        if(current_frame_->features_left_[i]->map_point_ && current_frame_->features_right_[i]) 
        {
            const cv::KeyPoint &kp_l = current_frame_->features_left_[i]->position_;
            const cv::KeyPoint &kp_r = current_frame_->features_right_[i]->position_;
            cv::Mat p_3d;
               
            Triangulate(kp_l, kp_r, P1, P2, p_3d);

            if(p_3d.at<float>(2) <= 0)
                continue;

            p_3d = R_wc * p_3d + t_wc;
            MapPoint* new_map_point = new MapPoint(p_3d);
            new_map_point->AddObservation(current_frame_->features_left_[i]);
            new_map_point->AddObservation(current_frame_->features_right_[i]);
            cnt_triangulated_pts++;
            map_->InsertMapPoint(new_map_point);
        }
    }
    std::cout << "New landmarks: " << cnt_triangulated_pts << std::endl;
    return cnt_triangulated_pts;
}

void FrontEnd::Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &p_3d)
{
    cv::Mat A(4,4,CV_32F);

    A.row(0) = kp1.pt.x * P1.row(2) - P1.row(0);
    A.row(1) = kp1.pt.y * P1.row(2) - P1.row(1);
    A.row(2) = kp2.pt.x * P2.row(2) - P2.row(0);
    A.row(3) = kp2.pt.y * P2.row(2) - P2.row(1);

    cv::Mat u,w,vt;
    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);
    p_3d = vt.row(3).t();
    p_3d = p_3d.rowRange(0,3)/p_3d.at<float>(3);
}

// Vec3d FrontEnd::WorldToCamera(const Vec3d &p_w, const SE3 &T_cw, const SE3 &ext) {
//     return ext * T_cw * p_w;
// }

// Vec2d FrontEnd::CameraToPixel(const Vec3d &p_c, const Mat33d &K) {
//     return Vec2d(
//         K(0, 0) * p_c(0, 0) / p_c(2, 0) + K(0, 2),
//         K(1, 1) * p_c(1, 0) / p_c(2, 0) + K(1, 2)
//     );
// }

cv::Point2f FrontEnd::WorldToPixelinLeft(const cv::Mat &p_w, const cv::Mat &T_cw, const cv::Mat &K)
{
    cv::Mat R_cw = T_cw.rowRange(0,3).colRange(0,3);
    cv::Mat t_cw = T_cw.rowRange(0,3).col(3);

    cv::Mat p_c = R_cw * p_w + t_cw;

    float u = K.at<float>(0, 0) * p_c.at<float>(0) / p_c.at<float>(2) + K.at<float>(0, 2);
    float v = K.at<float>(1, 1) * p_c.at<float>(0) / p_c.at<float>(2) + K.at<float>(1, 2);

    return cv::Point2f(u, v);
}

cv::Point2f FrontEnd::WorldToPixelinRight(const cv::Mat &p_w, const cv::Mat &T_cw, const cv::Mat &K, const float &bf)
{
    cv::Mat R_cw = T_cw.rowRange(0,3).colRange(0,3);
    cv::Mat t_cw = T_cw.rowRange(0,3).col(3);

    cv::Mat p_c = R_cw * p_w + t_cw;

    float u = (K.at<float>(0, 0) * p_c.at<float>(0) - bf) / p_c.at<float>(2) + K.at<float>(0, 2);
    float v = K.at<float>(1, 1) * p_c.at<float>(0) / p_c.at<float>(2) + K.at<float>(1, 2);

    return cv::Point2f(u, v);
}

bool FrontEnd::Reset()
{
    std::cout << "Reset is not implemented." << std::endl;
    return true;
}

}