#include "../inc/publisher.h"


FeaureTrackerPublisher::FeaureTrackerPublisher(ros::NodeHandle& nh,
                                                std::string topic_name,
                                                size_t buff_size)
                                                :nh_(nh){
    publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name, buff_size);
 }

void FeaureTrackerPublisher::publish(const cv::Mat &imgTrack, const double t){
    std_msgs::Header header;
    header.frame_id = "world";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    publisher_.publish(imgTrackMsg);
}