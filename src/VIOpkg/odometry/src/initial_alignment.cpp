#include "../inc/initial_alignment.h"
#include "../../parameters/src/parameters.h"
/**
 * @brief 用SfM的结果 和 粗糙的预积分结果 求解陀螺仪偏置，求解完成之后重新进行一次预积分
 * 
 * @param all_image_frame 
 * @param Bgs 
 */
void solveGyroscopeBias(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs)
{
    Matrix3d A;
    Vector3d b;
    Vector3d delta_bg;
    A.setZero();
    b.setZero();
    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++)
    {
        frame_j = next(frame_i);
        MatrixXd tmp_A(3, 3);
        tmp_A.setZero();
        VectorXd tmp_b(3);
        tmp_b.setZero();
        // 构造超定方程求解陀螺仪偏置
        // 1. 先获得SfM得到的帧间相对旋转
        Eigen::Quaterniond q_ij(frame_i->second.R.transpose() * frame_j->second.R);
        // 2. 超定方程的系数矩阵通过前面构造预积分残差模型的 误差传递雅克比 F 中的一小块直接获取
        tmp_A = frame_j->second.pre_integration->jacobian.template block<3, 3>(3, 12);
        // 3. 直接取四元数乘积的虚部的两倍作为李代数上的增量
        tmp_b = 2 * (frame_j->second.pre_integration->delta_q.inverse() * q_ij).vec();

        A += tmp_A.transpose() * tmp_A;
        b += tmp_A.transpose() * tmp_b;
    }
    // 求解超定方程
    std::cout << A << std::endl;
    delta_bg = A.ldlt().solve(b);
    ROS_WARN_STREAM("gyroscope bias initial calibration " << delta_bg.transpose());

    for (int i = 0; i <= windowSize; i++)
        Bgs[i] += delta_bg;

    // 修正偏置之后重新进行一次预积分
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end( ); frame_i++)
    {
        frame_j = next(frame_i);
        frame_j->second.pre_integration->repropagate(Vector3d::Zero(), Bgs[0]);
    }

}


bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x, Eigen::Vector3d &TIC)
{
    solveGyroscopeBias(all_image_frame, Bgs); // 用 SfM 的结果 联合 IMU的预积分 求解出陀螺仪的偏置 

    if(LinearAlignment(all_image_frame, g, x, TIC)) // 求解重力矢量、尺度因子、各帧速度
        return true;
    else 
        return false;
}

// 主要作用就是求解尺度因子 s 和 重力矢量 g
bool LinearAlignment(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x, Eigen::Vector3d &TIC)
{
    //all_image_frame 所有图像帧构成的map,图像帧保存了位姿，预积分量和关于角点的信息
    //g 重力加速度矢量
    //x 待优化变量，窗口中每帧的速度V[0:n]、重力g、尺度s
    int all_frame_count = all_image_frame.size();
    //优化量x的总维度
    int n_state = all_frame_count * 3 + 3 + 1;

    // 构造超定方程求解
    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    int i = 0;
    for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
    {
        frame_j = next(frame_i);

        MatrixXd tmp_A(6, 10);
        tmp_A.setZero();
        VectorXd tmp_b(6);
        tmp_b.setZero();

        double dt = frame_j->second.pre_integration->sum_dt;

        // 1. 位置预积分量对应 H 矩阵的部分
        tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
        tmp_A.block<3, 3>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity();
            // 这里为了求解过程中保持数值的稳定性，对 H 矩阵中和 尺度因子 s 相乘的块除以100，这样最后算出来的尺度因子会扩大100倍
        tmp_A.block<3, 1>(0, 9) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
        
        tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC;
        //cout << "delta_p   " << frame_j->second.pre_integration->delta_p.transpose() << endl;
        // 2. 速度预积分量对应 H 矩阵的部分
        tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
        tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
        tmp_A.block<3, 3>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity();
        
        tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v;
        //cout << "delta_v   " << frame_j->second.pre_integration->delta_v.transpose() << endl;

        Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
        //cov.block<6, 6>(0, 0) = IMU_cov[i + 1];
        //MatrixXd cov_inv = cov.inverse();
        cov_inv.setIdentity();

        MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
        VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

        A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
        b.segment<6>(i * 3) += r_b.head<6>();

        A.bottomRightCorner<4, 4>() += r_A.bottomRightCorner<4, 4>();
        b.tail<4>() += r_b.tail<4>();

        A.block<6, 4>(i * 3, n_state - 4) += r_A.topRightCorner<6, 4>();
        A.block<4, 6>(n_state - 4, i * 3) += r_A.bottomLeftCorner<4, 6>();
    }
    A = A * 1000.0;
    b = b * 1000.0;
    x = A.ldlt().solve(b);
    double s = x(n_state - 1) / 100.0; // 前面tmp_A.block<3, 1>(0, 9)构建的时候除以了100
    ROS_DEBUG("estimated scale: %f", s);

    // 这里得到的是参考帧的时刻的重力矢量，因为在上面求解超定方程的时候，涉及到的旋转变换都是相对于参考帧的
    g = x.segment<3>(n_state - 4);
    ROS_DEBUG_STREAM(" result g     " << g.norm() << " " << g.transpose());
    if(fabs(g.norm() - G.norm()) > 0.5 || s < 0)
    {
        return false;
    }
    // 进一步优化重力矢量
    // 在上一步中得到了一个先验值，这个真实的重力矢量的模长可以用先验值求得，但是方向不一定与先验的方向完全一致，所以要进行优化
    // 1. 取先验矢量正切空间上的两个正交向量，然后给这两个正交向量加一个扰动参数，最后把这三个向量进行加和作为新的重力矢量放到预积分等式中进行优化
    RefineGravity(all_image_frame, g, x, TIC);

    s = (x.tail<1>())(0) / 100.0;
    (x.tail<1>())(0) = s;
    ROS_DEBUG_STREAM(" refine     " << g.norm() << " " << g.transpose());
    if(s < 0.0 )
        return false;   
    else
        return true;
}


void RefineGravity(map<double, ImageFrame> &all_image_frame, Vector3d &g, VectorXd &x, Eigen::Vector3d &TIC)
{
    Vector3d g0 = g.normalized() * G.norm(); // 模长固定为 |G|
    Vector3d lx, ly;
    int all_frame_count = all_image_frame.size();
    int n_state = all_frame_count * 3 + 2 + 1;

    MatrixXd A{n_state, n_state};
    A.setZero();
    VectorXd b{n_state};
    b.setZero();

    map<double, ImageFrame>::iterator frame_i;
    map<double, ImageFrame>::iterator frame_j;
    for(int k = 0; k < 4; k++)  // 迭代 4 次
    {
        MatrixXd lxly(3, 2);
        lxly = TangentBasis(g0); // 获取正切空间上的两个正交向量组成的矩阵
        int i = 0;
        for (frame_i = all_image_frame.begin(); next(frame_i) != all_image_frame.end(); frame_i++, i++)
        {
            frame_j = next(frame_i);

            MatrixXd tmp_A(6, 9);
            tmp_A.setZero();
            VectorXd tmp_b(6);
            tmp_b.setZero();
            double dt = frame_j->second.pre_integration->sum_dt;

            tmp_A.block<3, 3>(0, 0) = -dt * Matrix3d::Identity();
            tmp_A.block<3, 2>(0, 6) = frame_i->second.R.transpose() * dt * dt / 2 * Matrix3d::Identity() * lxly;
            tmp_A.block<3, 1>(0, 8) = frame_i->second.R.transpose() * (frame_j->second.T - frame_i->second.T) / 100.0;     
            tmp_b.block<3, 1>(0, 0) = frame_j->second.pre_integration->delta_p + frame_i->second.R.transpose() * frame_j->second.R * TIC - TIC - frame_i->second.R.transpose() * dt * dt / 2 * g0;

            tmp_A.block<3, 3>(3, 0) = -Matrix3d::Identity();
            tmp_A.block<3, 3>(3, 3) = frame_i->second.R.transpose() * frame_j->second.R;
            tmp_A.block<3, 2>(3, 6) = frame_i->second.R.transpose() * dt * Matrix3d::Identity() * lxly;
            tmp_b.block<3, 1>(3, 0) = frame_j->second.pre_integration->delta_v - frame_i->second.R.transpose() * dt * Matrix3d::Identity() * g0;

            Matrix<double, 6, 6> cov_inv = Matrix<double, 6, 6>::Zero();
            cov_inv.setIdentity();

            MatrixXd r_A = tmp_A.transpose() * cov_inv * tmp_A;
            VectorXd r_b = tmp_A.transpose() * cov_inv * tmp_b;

            A.block<6, 6>(i * 3, i * 3) += r_A.topLeftCorner<6, 6>();
            b.segment<6>(i * 3) += r_b.head<6>();

            A.bottomRightCorner<3, 3>() += r_A.bottomRightCorner<3, 3>();
            b.tail<3>() += r_b.tail<3>();

            A.block<6, 3>(i * 3, n_state - 3) += r_A.topRightCorner<6, 3>();
            A.block<3, 6>(n_state - 3, i * 3) += r_A.bottomLeftCorner<3, 6>();
        }
        A = A * 1000.0;
        b = b * 1000.0;
        x = A.ldlt().solve(b);
        VectorXd dg = x.segment<2>(n_state - 3);
        g0 = (g0 + lxly * dg).normalized() * G.norm();
    }
    g = g0;
}

// 返回正切空间上的两个相互正交的向量
MatrixXd TangentBasis(Vector3d &g0)
{
    Vector3d b, c;
    Vector3d a = g0.normalized();
    Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b = (tmp - a * (a.transpose() * tmp)).normalized();
    c = a.cross(b);
    MatrixXd bc(3, 2);
    bc.block<3, 1>(0, 0) = b;
    bc.block<3, 1>(0, 1) = c;
    return bc;
}