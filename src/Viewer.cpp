#include "include/Viewer.h"

namespace usrl_vo {

Viewer::Viewer(Map* map):map_(map), current_frame_(NULL)
{
    viewer_thread_ = std::thread(std::bind(&Viewer::Run, this));
}

void Viewer::Close()
{
    viewer_running_ = false;
    viewer_thread_.join();
}

void Viewer::AddCurrentFrame(Frame* current_frame)
{
    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    current_frame_ = current_frame;
}

void Viewer::UpdateMap()
{
    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    assert(map_ != NULL);
    active_keyframes_ = map_->GetActiveKeyFrames();
    active_landmarks_ = map_->GetActiveMapPoints();
    map_updated_ = true;
}

void Viewer::Run()
{
    pangolin::CreateWindowAndBind("USRL-VO: Map Viewer",1024,768);

    glEnable(GL_DEPTH_TEST);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuMapPoints("menu.Map Points",true,true);

    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,2000,2000,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,-100,-0.1, 0,0,0,0.0,-1.0, 0.0));

    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while(!pangolin::ShouldQuit() && viewer_running_)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        // std::unique_lock<std::mutex> lock(viewer_data_mutex_);

        GetCurrentOpenGLCameraMatrix(Twc);

        if(menuFollowCamera)
        {
            s_cam.Follow(Twc);
        }

        DrawCurrentCamera(Twc);

        std::unique_lock<std::mutex> lock(viewer_data_mutex_);
        if(map_ && menuMapPoints)
        {
            DrawMapPoints();
        }

        cv::Mat im = DrawFrame();
        cv::imshow("USRL-VO: Current Frame", im);
        cv::waitKey(1); // 1e3/10.0

        pangolin::FinishFrame();
        usleep(3000);
    }

    std::cout << "Stop viewer." << std::endl;

}

void Viewer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    std::unique_lock<std::mutex> lock(viewer_data_mutex_);
    cv::Mat camera_pose = current_frame_->GetPose();
    if(!camera_pose.empty())
    {   
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
        {
            Rwc = camera_pose.rowRange(0, 3).colRange(0, 3).t();
            twc = -Rwc * camera_pose.rowRange(0, 3).col(3);
        }

        M.m[0] = Rwc.at<float>(0, 0);
        M.m[1] = Rwc.at<float>(1, 0);
        M.m[2] = Rwc.at<float>(2, 0);
        M.m[3] = 0.0;

        M.m[4] = Rwc.at<float>(0, 1);
        M.m[5] = Rwc.at<float>(1, 1);
        M.m[6] = Rwc.at<float>(2, 1);
        M.m[7] = 0.0;

        M.m[8] = Rwc.at<float>(0, 2);
        M.m[9] = Rwc.at<float>(1, 2);
        M.m[10] = Rwc.at<float>(2, 2);
        M.m[11] = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15] = 1.0;
    }
    else
        M.SetIdentity();
}

void Viewer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
{
    const float w = 0.7; //CameraSize
    const float line_width = 3.0;
    const float h = w * 0.75;
    const float z = w * 0.6;

    glPushMatrix();

    glMultMatrixd(Twc.m);

    glLineWidth(line_width);
    glColor3f(0.0f, 1.0f, 0.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();

}

void Viewer::DrawMapPoints()
{
    glPointSize(2);
    glBegin(GL_POINTS);
    glColor3f(0.0f, 0.0f, 0.0f);

    for(std::unordered_map<unsigned long, MapPoint*>::iterator mp_it = active_landmarks_.begin(); mp_it != active_landmarks_.end(); mp_it++)   
    {
        MapPoint* landmark = mp_it->second;
        cv::Mat pos = landmark->GetWorldPos();
        glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
    }
    glEnd();
}

cv::Mat Viewer::DrawFrame()
{
    cv::Mat im;
    current_frame_->left_img_.copyTo(im);

    if(im.channels() < 3)
    {
        cv::cvtColor(im, im, CV_GRAY2BGR);
    }

    const int n = current_frame_->features_left_.size();
    for(int i = 0; i < n; i++)
    {
        num_tracked_ = 0;
        num_rackedVO_ = 0;
        cv::Point2f pt1, pt2;
        Feature* feature = current_frame_->features_left_[i];
        pt1.x = feature->position_.pt.x - 5;
        pt1.y = feature->position_.pt.y - 5;
        pt2.x = feature->position_.pt.x + 5;
        pt2.y = feature->position_.pt.y + 5;

        if(feature->map_point_)
        {
            cv::rectangle(im, pt1, pt2, cv::Scalar(0, 255, 0));
            cv::circle(im, feature->position_.pt, 2, cv::Scalar(0, 255, 0), -1);
            num_tracked_++;
        }
        else
        {
            cv::rectangle(im, pt1, pt2, cv::Scalar(255, 0, 0));
            cv::circle(im, feature->position_.pt, 2, cv::Scalar(255, 0, 0), -1);
            num_rackedVO_++;
        }
    }

    return im;

    cv::Mat img_out;
    cv::cvtColor(current_frame_->left_img_, img_out, CV_GRAY2BGR);
    for (size_t i = 0; i < current_frame_->features_left_.size(); ++i)
    {
        if (current_frame_->features_left_[i]->map_point_)
        {
            Feature* feature = current_frame_->features_left_[i];
            cv::circle(img_out, feature->position_.pt, 2, cv::Scalar(0, 250, 0),
                       2);
        }
    }
    return img_out;
}

}