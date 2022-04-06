#pragma once
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

using namespace std;
class FeaureTrackerPublisher{
    public:
        using Ptr = shared_ptr<FeaureTrackerPublisher>;
        FeaureTrackerPublisher(ros::NodeHandle& nh, string topic_name, size_t buff_size);
        FeaureTrackerPublisher() = default;
        void publish(const cv::Mat &imgTrack, const double t);
    private:
        ros::NodeHandle nh_;
        ros::Publisher publisher_;
        std::string frameID_;
};

