#include "parameters.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <fstream>

using namespace std;

Parameters::Parameters(/* args */)
{
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
    MAX_CNT = parametersSettings["max_cnt"];
    MIN_DIST = parametersSettings["min_dist"];
    F_THRESHOLD = parametersSettings["F_threshold"];
    SHOW_TRACK = parametersSettings["show_track"];
    FLOW_BACK = parametersSettings["flow_back"];

    MULTIPLE_THREAD = parametersSettings["multiple_thread"];

    USE_IMU = parametersSettings["imu"];
    printf("USE_IMU: %d\n", USE_IMU);
    if(USE_IMU)
    {
        parametersSettings["imu_topic"] >> IMU_TOPIC;
        printf("IMU_TOPIC: %s\n", IMU_TOPIC.c_str());
        ACC_N = parametersSettings["acc_n"];
        ACC_W = parametersSettings["acc_w"];
        GYR_N = parametersSettings["gyr_n"];
        GYR_W = parametersSettings["gyr_w"];
        G.z() = parametersSettings["g_norm"];
    }

    SOLVER_TIME = parametersSettings["max_solver_time"];
    NUM_ITERATIONS = parametersSettings["max_num_iterations"];
    MIN_PARALLAX = parametersSettings["keyframe_parallax"];
    MIN_PARALLAX = MIN_PARALLAX / FOCAL_LENGTH;

    parametersSettings["output_path"] >> OUTPUT_FOLDER;
    VINS_RESULT_PATH = OUTPUT_FOLDER + "/vio.csv";
    std::cout << "result path " << VINS_RESULT_PATH << std::endl;
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

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
            EX_CALIB_RESULT_PATH = OUTPUT_FOLDER + "/extrinsic_parameter.csv";
        }
        if (ESTIMATE_EXTRINSIC == 0)
            ROS_WARN(" fix extrinsic param ");

        cv::Mat cv_T;
        parametersSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
    } 
    
    NUM_OF_CAM = parametersSettings["num_of_cam"];
    printf("camera number %d\n", NUM_OF_CAM);

    if(NUM_OF_CAM != 1 && NUM_OF_CAM != 2)
    {
        printf("num_of_cam should be 1 or 2\n");
        assert(0);
    }


    int pn = configFilePath.find_last_of('/');
    std::string configPath = configFilePath.substr(0, pn);
    
    std::string cam0Calib;
    parametersSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    CAM_NAMES.push_back(cam0Path);

    if(NUM_OF_CAM == 2)
    {
        STEREO = 1;
        std::string cam1Calib;
        parametersSettings["cam1_calib"] >> cam1Calib;
        std::string cam1Path = configPath + "/" + cam1Calib; 
        //printf("%s cam1 path\n", cam1Path.c_str() );
        CAM_NAMES.push_back(cam1Path);
        
        cv::Mat cv_T;
        parametersSettings["body_T_cam1"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        RIC.push_back(T.block<3, 3>(0, 0));
        TIC.push_back(T.block<3, 1>(0, 3));
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

    if(!USE_IMU)
    {
        ESTIMATE_EXTRINSIC = 0;
        ESTIMATE_TD = 0;
        printf("no imu, fix extrinsic param; no time offset calibration\n");
    }

    parametersSettings.release();
}