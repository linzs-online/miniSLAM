#include "../inc/subscriber.h"
#include <mutex>

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
    //ROS_INFO("000 : %d", img0_buf.size());
}

void ImageSubscriber::img1_callback(const sensor_msgs::ImageConstPtr &img1_msg){
    mutex_buf.lock();
    img1_buf.push(img1_msg);
    mutex_buf.unlock();
    //ROS_INFO("111 : %d", img1_buf.size());
}

ImageSubscriber::~ImageSubscriber(){

}