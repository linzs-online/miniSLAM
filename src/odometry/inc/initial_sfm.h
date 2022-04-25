#pragma once 
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <cstdlib>
#include <deque>
#include <map>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
using namespace Eigen;
using namespace std;


// 每个特征点都有一个 SFMFeature
struct SFMFeature
{
    bool state; // 特征点状态
    int id; // 特征点ID
    vector<pair<int,Vector2d>> observation;  // 所有共视到该特征点的图像帧ID 和 图像坐标
    double position[3];		// 该特征点在帧L坐标下的三维坐标
    double depth; 	// 该特征点的深度
};

struct ReprojectionError3D
{
	ReprojectionError3D(double observed_u, double observed_v)
		:observed_u(observed_u), observed_v(observed_v)
		{}

	template <typename T>
	bool operator()(const T* const camera_R,    // L帧到 i 帧的旋转 
					const T* const camera_T, 	// L帧到 i 帧的平移	
					const T* point,				// 3D空间点坐标，在L帧坐标系下
					T* residuals) const
	{
		T p[3];
		ceres::QuaternionRotatePoint(camera_R, point, p);	// 根据旋转矩阵和平移向量，估算出在第 i 帧下的空间坐标		
		p[0] += camera_T[0]; 
		p[1] += camera_T[1]; 
		p[2] += camera_T[2];
		// 归一化平面下的像素坐标
		T xp = p[0] / p[2];
    	T yp = p[1] / p[2];
    	residuals[0] = xp - T(observed_u); // 预测量 - 观测量
    	residuals[1] = yp - T(observed_v);
    	return true;
	}

	static ceres::CostFunction* Create(const double observed_x,
	                                   const double observed_y) 
	{
	  return (new ceres::AutoDiffCostFunction<
	          ReprojectionError3D, 2, 4, 3, 3>(
	          	new ReprojectionError3D(observed_x,observed_y)));
	}

	double observed_u;
	double observed_v;
};

class GlobalSFM
{
public:
	GlobalSFM();
	bool construct(int frame_num, Quaterniond* q, Vector3d* T, int l,
			  const Matrix3d relative_R, const Vector3d relative_T,
			  vector<SFMFeature> &sfm_f, map<int, Vector3d> &sfm_tracked_points);

private:
	// PnP得到当前帧相对与 参考帧l 的位姿
	bool solveFrameByPnP(Matrix3d &R_initial, Vector3d &P_initial, int i, vector<SFMFeature> &sfm_f);
	// 三角化两个点
	void triangulatePoint(Eigen::Matrix<double, 3, 4> &Pose0, Eigen::Matrix<double, 3, 4> &Pose1,
							Vector2d &point0, Vector2d &point1, Vector3d &point_3d);
	// 三角化两帧
	void triangulateTwoFrames(int frame0, Eigen::Matrix<double, 3, 4> &Pose0, 
							  int frame1, Eigen::Matrix<double, 3, 4> &Pose1,
							  vector<SFMFeature> &sfm_f);

	int feature_num;
};