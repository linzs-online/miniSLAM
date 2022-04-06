#pragma once
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <std_msgs/String.h>

using namespace std;
class ImageSubscriber
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber0_;
    ros::Subscriber subscriber1_;
    void img0_callback(const sensor_msgs::ImageConstPtr &img0_msg);
    void img1_callback(const sensor_msgs::ImageConstPtr &img1_msg);

public:
    using Ptr = std::shared_ptr<ImageSubscriber>;
    queue<sensor_msgs::ImageConstPtr> img0_buf;
    queue<sensor_msgs::ImageConstPtr> img1_buf;
    ImageSubscriber(ros::NodeHandle& nh, std::string topic0_name, std::string topic1_name, size_t buff_size);
    ImageSubscriber() = default;
    ~ImageSubscriber();
};

