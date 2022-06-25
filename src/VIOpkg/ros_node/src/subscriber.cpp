#include "../inc/subscriber.h"
#include <mutex>
#include <ros/transport_hints.h>

std::mutex mutex_buf;
ImageSubscriber::ImageSubscriber(ros::NodeHandle& nh, 
                                    std::string topic0_name,
                                    std::string topic1_name, 
                                    size_t buff_size):nh_(nh){
    subscriber0_ = nh_.subscribe(topic0_name, buff_size, &ImageSubscriber::img0_callback, this);
    subscriber1_ = nh_.subscribe(topic1_name, buff_size, &ImageSubscriber::img1_callback, this);
}

void ImageSubscriber::img0_callback(const sensor_msgs::ImageConstPtr &img0_msg){
    mutex_buf.lock();
    img0_buf.push(img0_msg);
    mutex_buf.unlock();
    //ROS_INFO("img0_buf Size  : %d", (int)img0_buf.size());
}

void ImageSubscriber::img1_callback(const sensor_msgs::ImageConstPtr &img1_msg){
    mutex_buf.lock();
    img1_buf.push(img1_msg);
    mutex_buf.unlock();
    //ROS_INFO("img1_buf Size : %d", (int)img1_buf.size());
}

ImageSubscriber::~ImageSubscriber(){

}

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

IMU_subscriber::IMU_subscriber(ros::NodeHandle& nh, std::string topic_name, size_t buff_size):nh_(nh)
{   
    // NOTE tcpNoDelay: a TCP transport is used, specifies whether or not to use TCP_NODELAY to provide a potentially lower-latency connection. 
    subscriber_ = nh_.subscribe(topic_name, buff_size, &IMU_subscriber::imu_callback, this, ros::TransportHints().tcpNoDelay());
}

void IMU_subscriber::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    mutex_buf.lock();
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    acc_buf.push(make_pair(t, acc));
    gyr_buf.push(make_pair(t, gyr));
    mutex_buf.unlock();
    //ROS_INFO("imu_buf Size : %d", (int)acc_buf.size());
    return;
}