#include "parameters.h"
#include <ros/ros.h>
#include <fstream>

using namespace std;

Parameters::Parameters(std::string configFilePath)
{
    readParameters(configFilePath);
}

Parameters::~Parameters()
{
}

void Parameters::readParameters(std::string configFilePath){
    FILE *fh = fopen(configFilePath.c_str(),"r");
    if(fh == NULL){
        ROS_WARN("config_file dosen't exist; wrong config_file path");
        ROS_BREAK(); // 用于中断程序并输出本句所在文件/行数
        return;
    }
    fclose(fh);

    cv::FileStorage parametersSettings(configFilePath, cv::FileStorage::READ);
    if(!parametersSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }
    parametersSettings["image0_topic"] >> IMAGE0_TOPIC;
    parametersSettings["image1_topic"] >> IMAGE1_TOPIC;
    parametersSettings["imu_topic"] >> IMU_TOPIC;
    ROS_INFO("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());

    {
        cv::Mat cv_DistCoeff_0;
        parametersSettings["DistCoeff_0"] >> cv_DistCoeff_0;
        // Eigen::Matrix<double, 1, 5> DistCoeff_0;
        // cv::cv2eigen(cv_DistCoeff_0, DistCoeff_0);
        distCoeffs.emplace_back(cv_DistCoeff_0);
        
        cv::Mat cv_CameraMat_0;
        parametersSettings["CameraMat_0"] >> cv_CameraMat_0;
        // Eigen::Matrix3d CameraMat_0;
        // cv::cv2eigen(cv_CameraMat_0, CameraMat_0);
        cameraMatrix.emplace_back(cv_CameraMat_0);
    }
    
    {
        cv::Mat cv_DistCoeff_1;
        parametersSettings["DistCoeff_1"] >> cv_DistCoeff_1;
        // Eigen::Matrix<double, 1, 5> DistCoeff_1;
        // cv::cv2eigen(cv_DistCoeff_1, DistCoeff_1);
        distCoeffs.emplace_back(cv_DistCoeff_1);
        
        cv::Mat cv_CameraMat_1;
        parametersSettings["CameraMat_1"] >> cv_CameraMat_1;
        // Eigen::Matrix3d CameraMat_1;
        // cv::cv2eigen(cv_CameraMat_1, CameraMat_1);
        cameraMatrix.emplace_back(cv_CameraMat_1);
    }

    MAX_CNT = parametersSettings["max_cnt"];
    MIN_DIST = parametersSettings["min_dist"];
    F_THRESHOLD = parametersSettings["F_threshold"];
    SHOW_TRACK = parametersSettings["show_track"];
    FLOW_BACK = parametersSettings["flow_back"];


    ACC_N = parametersSettings["acc_n"];
    ACC_W = parametersSettings["acc_w"];
    GYR_N = parametersSettings["gyr_n"];
    GYR_W = parametersSettings["gyr_w"];
    G.z() = parametersSettings["g_norm"];

    SOLVER_TIME = parametersSettings["max_solver_time"];
    NUM_ITERATIONS = parametersSettings["max_num_iterations"];
    MIN_PARALLAX = parametersSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    ESTIMATE_EXTRINSIC = parametersSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 2)
    {
        ROS_WARN("have no prior about extrinsic param, calibrate extrinsic param");
        RIC.push_back(Eigen::Matrix3d::Identity());
        TIC.push_back(Eigen::Vector3d::Zero());
        EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
    }
    else 
    {
        if ( ESTIMATE_EXTRINSIC == 1)
        {
            ROS_WARN(" Optimize extrinsic param around initial guess!");
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T0;
        parametersSettings["T_ic0"] >> cv_T0;
        Eigen::Matrix4d T0;
        cv::cv2eigen(cv_T0, T0);
        RIC.push_back(T0.block<3, 3>(0, 0));
        TIC.push_back(T0.block<3, 1>(0, 3));

        cv::Mat cv_T1;
        parametersSettings["T_ic1"] >> cv_T1;
        Eigen::Matrix4d T1;
        cv::cv2eigen(cv_T1, T1);
        RIC.push_back(T1.block<3, 3>(0, 0));
        TIC.push_back(T1.block<3, 1>(0, 3));
    } 
    
    INIT_DEPTH = 5.0;
    BIAS_ACC_THRESHOLD = 0.1;
    BIAS_GYR_THRESHOLD = 0.1;

    TD = parametersSettings["td"];
    ESTIMATE_TD = parametersSettings["estimate_td"];
    if (ESTIMATE_TD)
        ROS_INFO_STREAM("Unsynchronized sensors, online estimate time offset, initial td: " << TD);
    else
        ROS_INFO_STREAM("Synchronized sensors, fix time offset: " << TD);

    ROW = parametersSettings["image_height"];
    COL = parametersSettings["image_width"];
    ROS_INFO("ROW: %d COL: %d ", ROW, COL);

    parametersSettings.release();
}