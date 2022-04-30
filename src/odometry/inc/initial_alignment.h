#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>

#include "utility.h"
#include <ros/ros.h>
#include <map>
#include "feature_manager.h"
#include "pre_integrated.h"

using namespace Eigen;
using namespace std;
// 每张图片都有一个这样的数据结构，里面包含了特征点的信息，以及拍摄到该帧图片的相机位姿
class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const FeaturePointMap& _featurePoints, double _t):
            t{_t},is_key_frame{false},featurePoints(_featurePoints){};
        FeaturePointMap featurePoints;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase* pre_integration;
        bool is_key_frame;
};
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs);
bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);
bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x);
void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x);
MatrixXd TangentBasis(Vector3d &g0);