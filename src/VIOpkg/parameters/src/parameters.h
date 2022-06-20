#pragma once
#include <iostream>
#include <map>
#include <eigen3/Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <memory>

using namespace std;
using Vector3d = Eigen::Matrix<double, 3, 1>;
using Vector2d = Eigen::Matrix<double, 2, 1>;
using Matrix3d = Eigen::Matrix<double, 3, 3>;
using Quaterniond = Eigen::Quaternion<double>;
using MatrixXd = Eigen::Matrix<double, -1, -1>;


const int windowSize = 10;
const double FOCAL_LENGTH = 460.0;
Eigen::Vector3d G{0.0, 0.0, 9.8};
class Parameters
{
private:
    /* data */
public:
    using Ptr = std::shared_ptr<Parameters>;
    Parameters(std::string configFilePath);
    void readParameters(std::string configFilePath);
    ~Parameters();

    const double FOCAL_LENGTH = 460.0;
    const int WINDOW_SIZE = 10;
    const int NUM_OF_F = 1000;

    std::string IMAGE0_TOPIC, IMAGE1_TOPIC; //话题映射
    double INIT_DEPTH;  // 每个点的初始深度
    double MIN_PARALLAX; // 最小视差

    double ACC_N, ACC_W;    // 加速度计的白噪声和随机游走噪声
    double GYR_N, GYR_W;    // 陀螺仪计的白噪声和随机游走噪声

    std::vector<Eigen::Matrix3d> RIC;   // 外参矩阵
    std::vector<Eigen::Vector3d> TIC;

    std::vector<cv::Mat> distCoeffs; 
    std::vector<cv::Mat> cameraMatrix;

    Eigen::Vector3d G{0.0, 0.0, 9.8};   

    double BIAS_ACC_THRESHOLD;
    double BIAS_GYR_THRESHOLD;
    double SOLVER_TIME;
    int NUM_ITERATIONS;
    int ESTIMATE_EXTRINSIC;
    int ESTIMATE_TD;
    int ROLLING_SHUTTER;
    std::string EX_CALIB_RESULT_PATH;
    std::string VINS_RESULT_PATH;
    std::string OUTPUT_FOLDER;
    std::string IMU_TOPIC;
    int ROW, COL;
    double TD;
    int NUM_OF_CAM;
    int STEREO;
    int USE_IMU;
    int MULTIPLE_THREAD;
    std::map<int, Eigen::Vector3d> pts_gt;
    std::string FISHEYE_MASK;
    std::vector<std::string> CAM_NAMES;
    int MAX_CNT;
    int MIN_DIST;
    double F_THRESHOLD;
    int SHOW_TRACK;
    int FLOW_BACK;
};


