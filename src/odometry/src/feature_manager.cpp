#include "../inc/feature_manager.h"

#include <ros/ros.h>

FeatureManager::FeatureManager(Matrix3d _Rs[], Parameters::Ptr _paramPtr):Rs(_Rs),paramPtr(_paramPtr)
{
    for (auto i = 0; i < 2; i++)
        R_c2i[i].setIdentity();
}

int FeatureManager::getFeatureCount()
{
    int cnt = 0;
    for (auto &it : featureList)
    {   
        it.used_num = it.feature_per_frame.size();
        // 如果这个特征点在两帧上被观测到了，并且第一次观测到的帧数不是最后，说明这个特征点有效
        if (it.used_num >= 4 && it.start_frame < windowSize - 2)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frameCount, const FeatureMap &featureMap, double td)
{
    ROS_DEBUG("input feature: %d", (int)featureMap.size());  // 该帧特征点的数量
    ROS_DEBUG("num of feature: %d", getFeatureCount());      // 该帧中有效的特征点个数

    double parallax_sum = 0;    // 所有特征点的视差总和
    int parallax_num = 0;   
    last_track_num = 0;
    new_feature_num = 0;
    long_track_num = 0;

    // 1. 把所有特征点放入到 featureList中
    for(auto &id_pts : featureMap)
    {   // 特征点在每帧中的信息，归一化平面坐标、像素坐标、归一化平面上的速度
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        // 赋值入右目的信息
        f_per_fra.rightObservation(id_pts.second[1].second);

        int feature_id = id_pts.first;
        // 自定义查找函数，返回在feature中的迭代器
        auto it = find_if(featureList.begin(), featureList.end(), 
                            [feature_id](const FeaturePerId &it){
                                return it.feature_id == feature_id;
                            });
        // 找不到，说明这个点还没记录在 feature 里面，现在把它添加进去，并统计数目
        if (it == featureList.end())
        {
            featureList.push_back(FeaturePerId(feature_id, frameCount));
            featureList.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {   // 记录追踪到的特征点在各帧中的特征信息
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;     // 此帧有多少相同的特征点被跟踪
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;   // 追踪次数超过四次，说明该点长时间成功追踪
        }
    }
    // 2. 追踪次数小于20次或者窗口里面关键帧的数量很少，那么说明这个也是关键帧
    if (frameCount < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true; // 说明当前帧是新的关键帧，直接返回

    // 3. 计算每个特征在次新帧中和次次新帧中的视差
    for (auto &it_per_id : featureList)
    {
        if (it_per_id.start_frame <= frameCount - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frameCount - 1)
        {
            // 总视差：该特征点在两帧归一化平面上坐标点的距离
            parallax_sum += compensatedParallax2(it_per_id, frameCount);
            parallax_num++;
        }
    }
    // 第一次加进去的，是关键帧
    if (parallax_num == 0)
    {
        return true;
    }
    else
    {
        ROS_DEBUG("parallax_sum: %lf, parallax_num: %d", parallax_sum, parallax_num);
        ROS_DEBUG("current average parallax: %lf", parallax_sum / parallax_num * FOCAL_LENGTH);
        return parallax_sum / parallax_num >= paramPtr->MIN_PARALLAX;
    }
}

double FeatureManager::compensatedParallax2(const FeaturePerId &it_per_id, int frame_count)
{
    const FeaturePerFrame &frame_i = it_per_id.feature_per_frame[frame_count - 2 - it_per_id.start_frame];
    const FeaturePerFrame &frame_j = it_per_id.feature_per_frame[frame_count - 1 - it_per_id.start_frame];

    double ans = 0;
    Vector3d p_j = frame_j.point;

    double u_j = p_j(0);
    double v_j = p_j(1);

    Vector3d p_i = frame_i.point;

    double dep_i = p_i(2);
    double u_i = p_i(0) / dep_i;
    double v_i = p_i(1) / dep_i;
    double du = u_i - u_j, dv = v_i - v_j;

    ans = max(ans, sqrt(du * du + dv * dv));
    return ans;
}

// 获得指定两帧的共视特征点3D坐标
vector<pair<Vector3d, Vector3d>> FeatureManager::getCorresponding(int frame_count_l, int frame_count_r)
{
    vector<pair<Vector3d, Vector3d>> corres;
    // 遍历所有特征点
    for (auto &it : featureList)
    {   // 1. 首先保证跟踪到这个特征点的起始帧和最后一帧在我们指定的两帧范围之内
        if (it.start_frame <= frame_count_l && it.endFrame() >= frame_count_r)
        {   // 2. 获得给定两帧对当前特征点的3D坐标
            Vector3d a = Vector3d::Zero(), b = Vector3d::Zero();
            int idx_l = frame_count_l - it.start_frame;
            int idx_r = frame_count_r - it.start_frame;

            a = it.feature_per_frame[idx_l].point;

            b = it.feature_per_frame[idx_r].point;
            
            corres.push_back(make_pair(a, b));
        }
    }
    return corres;
}

