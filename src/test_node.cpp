#include "parameters/src/parameters.h"
#include "odometry/inc/publisher.h"
#include "odometry/inc/feature_tracker.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <mutex>
#include <thread>
using namespace std;
std::mutex m_buf;



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

void sync_process(ImageSubscriber::Ptr &imageSubscriber_Ptr,
                    FeatureTracker::Ptr &featureTracker_Ptr,
                    FeaureTrackerPublisher::Ptr &featurePub_Ptr){
    sleep(1); // 等待ROS消息队列建立
    while (1)
    {
        cv::Mat image0, image1;
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!imageSubscriber_Ptr->img0_buf.empty() && !imageSubscriber_Ptr->img1_buf.empty())
        {
            double time0 = imageSubscriber_Ptr->img0_buf.front()->header.stamp.toSec();
            double time1 = imageSubscriber_Ptr->img1_buf.front()->header.stamp.toSec();
            // 0.003s sync tolerance
            if(time0 < time1 - 0.003)
            {
                imageSubscriber_Ptr->img0_buf.pop();
                printf("throw img0\n");
            }
            else if(time0 > time1 + 0.003)
            {
                imageSubscriber_Ptr->img1_buf.pop();
                printf("throw img1\n");
            }
            else
            {
                time = imageSubscriber_Ptr->img0_buf.front()->header.stamp.toSec();
                header = imageSubscriber_Ptr->img0_buf.front()->header;
                image0 = getImageFromMsg(imageSubscriber_Ptr->img0_buf.front());
                imageSubscriber_Ptr->img0_buf.pop();
                image1 = getImageFromMsg(imageSubscriber_Ptr->img1_buf.front());
                imageSubscriber_Ptr->img1_buf.pop();
                printf("find img0 and img1\n");
            }
        }
        m_buf.unlock();
        //ROS_INFO("%d  %d", imageSubscriber_Ptr->img0_buf.size(), imageSubscriber_Ptr->img1_buf.size());
        if(!image0.empty() || !image1.empty())
            featureTracker_Ptr->featureFrame = featureTracker_Ptr->trackImage(time, image0, image1);
        // cv::imshow("特征点提取", featureTracker_Ptr->imTrack);
        // cv::waitKey(1);

        // 把特征点图片发布出去，用于可视化
        featurePub_Ptr->publish(featureTracker_Ptr->imTrack, time);
        
        // 把特征点存到特征缓冲队列里面，供后面使用IMU融合
        if(featureTracker_Ptr->inputImageCnt % 2 == 0)
        {
            m_buf.lock();
            featureTracker_Ptr->featureBuf.push(make_pair(time, featureTracker_Ptr->featureFrame));
            m_buf.unlock();
        }
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "my_estimator");
    ros::NodeHandle nh;
    
    string configFilePath = argv[1];
    cout << "configFilePath: " << configFilePath << endl;
    // 读取参数类
    Parameters::Ptr parameters_ptr = make_shared<Parameters>();
    parameters_ptr->readParameters(configFilePath);
    
    // 注册特征发布ROS 线程
    FeaureTrackerPublisher::Ptr featureTrackerPub_ptr = make_shared<FeaureTrackerPublisher>(nh, "image_track", 1000);
    // 注册图像订阅ROS 线程
    ImageSubscriber::Ptr imageSubscriber_ptr = make_shared<ImageSubscriber>(nh, parameters_ptr->IMAGE0_TOPIC, 
                                                                                    parameters_ptr->IMAGE1_TOPIC, 100);
    IMU_subscriber::Ptr ImuSub_Ptr = make_shared<IMU_subscriber>(nh, parameters_ptr->IMU_TOPIC, 2000);
    // 图像特征提取类
    FeatureTracker::Ptr featureTracker_ptr = make_shared<FeatureTracker>(parameters_ptr);

    // 位姿估计线程
    std::thread featureTrackThread(sync_process, ref(imageSubscriber_ptr),
                                                 ref(featureTracker_ptr),
                                                 ref(featureTrackerPub_ptr));
    ros::spin();
    
    int count = 0;
    while (ros::ok())
    {
       
        ros::spinOnce();
        ROS_INFO("****************************%d", imageSubscriber_ptr->img0_buf.size());
        //loop_rate.sleep();
        ++count;
    }
    ros::shutdown();
    return 0;
}
