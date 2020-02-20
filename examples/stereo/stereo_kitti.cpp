#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<include/VisualOdometry.h>

void LoadImages(const std::string &path_sequence, std::vector<std::string> &image_left,
                std::vector<std::string> &image_right, std::vector<double> &time_stamps);

int main(int argc, char **argv) {
    if(argc != 3) {
        std::cerr << std::endl << "Usage: ./stereo_kitti path_to_settings path_to_sequence" << std::endl;
    }

    std::vector<std::string> image_left;
    std::vector<std::string> image_right;
    std::vector<double> time_stamps;
    LoadImages(std::string(argv[2]), image_left, image_right, time_stamps);

    const int num_images = image_left.size();

    usrl_vo::VisualOdometry VO(argv[1], usrl_vo::VisualOdometry::STEREO, true);

    std::vector<float> time_vo;
    time_vo.resize(num_images);

    cv::Mat im_left, im_right;
    for(int i = 0; i < num_images; i++) {
        im_left = cv::imread(image_left[i], CV_LOAD_IMAGE_UNCHANGED);
        im_right = cv::imread(image_right[i], CV_LOAD_IMAGE_UNCHANGED);
        double time_frame = time_stamps[i];

        if(image_left.empty()) {
            std::cerr << std::endl << "Failed to load image at: "
                      << std::string(image_left[i]) << std::endl;
            return 1;
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        VO.RunStereo(im_left, im_right, time_frame);

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 -t1).count();

        time_vo[i] = time_used;

        double T = 0;
        if(i < num_images - 1) {
            T = time_stamps[i + 1] - time_frame;
        } else if(i > 0) {
            T = time_frame - time_stamps[i - 1];
        }

        if(time_used < T) usleep((T-time_used)*1e-6);
    }

    VO.Shutdown();

    return 0;
}

void LoadImages(const std::string &path_sequence, std::vector<std::string> &image_left,
                std::vector<std::string> &image_right, std::vector<double> &time_stamps)
{
    std::ifstream f_times;
    std::string time_file = path_sequence + "/times.txt";
    f_times.open(time_file.c_str());
    while(!f_times.eof())
    {
        std::string s;
        getline(f_times,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            ss >> t;
            time_stamps.push_back(t);
        }
    }

    std::string pre_left = path_sequence + "/image_0/";
    std::string pre_right = path_sequence + "/image_1/";

    const int num_times = time_stamps.size();
    image_left.resize(num_times);
    image_right.resize(num_times);

    for(int i = 0; i < num_times; i++)
    {
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << i;
        image_left[i] = pre_left + ss.str() + ".png";
        image_right[i] = pre_right + ss.str() + ".png";
    }
}