#include <ros/ros.h>
#include "parameters/src/parameters.h"
#include "ros_node/inc/subscriber.h"
#include "ros_node/inc/publisher.h"
#include "feature_tracker/inc/feature_tracker.h"
#include <mutex>
#include <thread>
using namespace std;
std::mutex m_buf;

/**
 * @brief 根据订阅图片构建featureQueue特征点队列
 * 
 * @param imageSubscriber_Ptr 
 * @param featureTracker_Ptr 
 * @param featurePub_Ptr 
 */
void processImages(ImageSubscriber::Ptr &imageSubscriber_Ptr,
                    FeatureTracker::Ptr &featureTracker_Ptr,
                    FeaureTrackerPublisher::Ptr &featurePub_Ptr){
    sleep(1); // 等待ROS消息队列建立
    while (1){
        cv::Mat image0, image1;
        std_msgs::Header header;
        double time = 0;
        m_buf.lock();
        if (!imageSubscriber_Ptr->img0_buf.empty() && !imageSubscriber_Ptr->img1_buf.empty()){
            double time0 = imageSubscriber_Ptr->img0_buf.front()->header.stamp.toSec();
            double time1 = imageSubscriber_Ptr->img1_buf.front()->header.stamp.toSec();
            // 0.003s sync tolerance
            if(time0 < time1 - 0.003){
                imageSubscriber_Ptr->img0_buf.pop();
                printf("throw img0\n");
            }
            else if(time0 > time1 + 0.003){
                imageSubscriber_Ptr->img1_buf.pop();
                printf("throw img1\n");
            }
            else{
                time = imageSubscriber_Ptr->img0_buf.front()->header.stamp.toSec();
                header = imageSubscriber_Ptr->img0_buf.front()->header;
                image0 = getImageFromMsg(imageSubscriber_Ptr->img0_buf.front());
                imageSubscriber_Ptr->img0_buf.pop();
                image1 = getImageFromMsg(imageSubscriber_Ptr->img1_buf.front());
                imageSubscriber_Ptr->img1_buf.pop();
                ROS_INFO("find img0 and img1\n");
            }
        }
        m_buf.unlock();
        if(!image0.empty() || !image1.empty())
            featureTracker_Ptr->featurePointMap = featureTracker_Ptr->trackImage(time, image0, image1); // 进行特征提取
        
        featurePub_Ptr->publish(featureTracker_Ptr->imTrack, time);  // 把特征点图片发布出去，用于RVIZ
        
        // 把特征点存到特征缓冲队列里面，供后面使用IMU融合，隔两帧采样一次，低频采样
        if(featureTracker_Ptr->inputImageCnt % 2 == 0){
            m_buf.lock();
            /****************************特征点队列***************************/
            // 一个时刻 对应 一张充满特征点的图
            featureTracker_Ptr->t_featureQueue.emplace(make_pair(time, featureTracker_Ptr->featurePointMap));
            /****************************特征点队列***************************/
            m_buf.unlock();
        }
        ROS_INFO("FeatureTracker Finish!");
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}

// /**
//  * @brief 根据特征点、陀螺仪数据优化位姿
//  * 
//  * @param featureTracker_Ptr 
//  * @param IMU_sub_Ptr 
//  * @param preIntegrated_Ptr 
//  * @param estimator_Ptr 
//  */
// void processMeasurements(FeatureTracker::Ptr &featureTracker_Ptr,
//                             ImageSubscriber::Ptr &IMU_sub_Ptr,
//                             PreIntegrated::Ptr &preIntegrated_Ptr,
//                             Estimator::Ptr &estimator_Ptr){
//     while (1){
//         pair<double, FeaturePointMap> t_featurePointMap;
//         vector<pair<double, Eigen::Vector3d>> accVector, gyrVector;

//         if(!featureTracker_Ptr->t_featureQueue.empty()){
//             // 1. 取出当前要处理的特征图
//             t_featurePointMap = featureTracker_Ptr->t_featureQueue.front();
//             preIntegrated_Ptr->curTime = t_featurePointMap.first + estimator_Ptr->td;
//             while (1){   
//                 // 检查IMU 队列是否准备好
//                 if (preIntegrated_Ptr->IMUAvailable(preIntegrated_Ptr->curTime))
//                     break;
//                 else
//                 {
//                     printf("wait for imu ... \n");
//                     std::chrono::milliseconds dura(5);
//                     std::this_thread::sleep_for(dura);
//                 }
//             }
//             m_buf.lock();
//             // 获取要预积分的数组，存放到accVector，gyrVector
//             preIntegrated_Ptr->getIMUInterVal(preIntegrated_Ptr->prevTime, preIntegrated_Ptr->curTime, accVector, gyrVector);
//             featureTracker_Ptr->t_featureQueue.pop();
//             m_buf.unlock();

//             if(!estimator_Ptr->initFirstPoseFlag)
//             {   // 初始化世界坐标系原点
//                 estimator_Ptr->initFirstIMUPose(accVector);  // 初始化Rs[0]，初始姿态到世界坐标系的变换
//                 estimator_Ptr->initFirstPoseFlag = true;
//             }
//             // 2. 进行预积分，求取 Rs，Vs，Ps
//             preIntegrated_Ptr->prevIntegrated(estimator_Ptr->frameCount, accVector, gyrVector);
            
//         }
//         m_buf.lock();

//         // 3. 根据特征图和预积分结果优化位姿
//         estimator_Ptr->poseEstimation(t_featurePointMap);

//         // 发布优化结果
//         //publishResult();

//         std::chrono::milliseconds dura(2);
//         std::this_thread::sleep_for(dura);
//     }
    
// }

int main(int argc, char * argv[])
{
    ros::init(argc, argv, "my_estimator");
    ros::NodeHandle nh;
    
    // string configFilePath = argv[1];
    string configFilePath = "/home/linzs/myCode/SLAMcode/miniSLAM_ws/config/stereo_imu_config.yaml";
    ROS_INFO("configFilePath: %s ", configFilePath.c_str());

    // 初始化读取参数类
    Parameters::Ptr parameters_ptr = make_shared<Parameters>(configFilePath);
    
    // 初始化图像特征提取类
    FeatureTracker::Ptr featureTracker_ptr = make_shared<FeatureTracker>(parameters_ptr);

    // // 注册特征发布ROS 线程
    FeaureTrackerPublisher::Ptr featureTrackerPub_ptr = make_shared<FeaureTrackerPublisher>(nh, "image_track", 1000);
    // 注册图像订阅ROS 线程
    ImageSubscriber::Ptr imageSub_ptr = make_shared<ImageSubscriber>(nh, parameters_ptr->IMAGE0_TOPIC, 
                                                                         parameters_ptr->IMAGE1_TOPIC, 100);
    
    // ros::Publisher imgPub0 = nh.advertise<sensor_msgs::Image>("rawImg",100);
    // ros::Publisher imgPub1 = nh.advertise<sensor_msgs::Image>("dealImg",100);

    // // 注册IMU订阅ROS 线程
    // IMU_subscriber::Ptr ImuSub_Ptr = make_shared<IMU_subscriber>(nh, parameters_ptr->IMU_TOPIC, 2000);
    
    // // 特征提取线程
    std::thread featureTrackThread(processImages, ref(imageSub_ptr),
                                                  ref(featureTracker_ptr),
                                                  ref(featureTrackerPub_ptr));

    // Estimator estimator(parameters_ptr); // 初始化位姿估计器
    // 处理测量值线程
    // std::thread odometryThread();
    ros::spin();
    while (ros::ok())
    {
        // std_msgs::Header header;
        // header.frame_id = "world";
        // header.stamp = ros::Time(count);
        // if(!imageSub_ptr->img0_buf.empty()){
        //     cv::Mat imgTest = getImageFromMsg(imageSub_ptr->img0_buf.front());
        //     sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "mono8", imgTest).toImageMsg();
        //     imgPub0.publish(imgTrackMsg);

        //     cv::Mat okimg;
        //     cv::undistort(imgTest, okimg, parameters_ptr->cameraMatrix[0], parameters_ptr->distCoeffs[0]);
        //     sensor_msgs::ImagePtr imgTrackMsg1 = cv_bridge::CvImage(header, "mono8", okimg).toImageMsg();
        //     imgPub1.publish(imgTrackMsg1);

        //     ROS_INFO("rows size of pic %d", okimg.rows);
        //     ROS_INFO("cols size of pic %d", okimg.cols);
        //     ros::spinOnce();
        //     imageSub_ptr->img0_buf.pop();
        // }
        // else
        //     ros::spinOnce();
    }
    ros::shutdown();
    return 0;
}
