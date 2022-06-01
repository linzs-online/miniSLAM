#pragma once
#include <ros/ros.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <Eigen/Core>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

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

/**
 * @brief 从标准的ROS消息转转换成图片
 * 
 * @param img_msg 
 * @return cv::Mat 
 */
cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();
    return img;
}


/**
 * @brief 订阅IMU信息
 * 
 */
class IMU_subscriber
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber subscriber_;
    void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);

public:
    using Ptr = shared_ptr<IMU_subscriber>;
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    queue<pair<double, Vector3d>> acc_buf;
    queue<pair<double, Vector3d>> gyr_buf;
    IMU_subscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size);
};



