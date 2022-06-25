#pragma once
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

using namespace std;

class Publisher{
    private:
        ros::NodeHandle nh_;
        ros::Publisher pub_imageTrack;
        ros::Publisher pub_odometry;
    public:
        using Ptr = shared_ptr<Publisher>;
        Publisher(ros::NodeHandle &nh);
        void regsiterPublisher();
        void pubImageTrack(const cv::Mat &imgTrack, const double t);
        ~Publisher();
};

