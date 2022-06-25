#include "../inc/publisher.h"


Publisher::Publisher(ros::NodeHandle &nh):nh_(nh){
    regsiterPublisher();
}


void Publisher::regsiterPublisher(){
    pub_imageTrack = nh_.advertise<sensor_msgs::Image>("image_track", 1000);
    pub_odometry = nh_.advertise<nav_msgs::Odometry>("odometry", 1000);
}


void Publisher::pubImageTrack(const cv::Mat &imgTrack, const double t){
    std_msgs::Header header;
    header.frame_id = "imgTrack";
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_imageTrack.publish(imgTrackMsg);
}


Publisher::~Publisher(){

}
