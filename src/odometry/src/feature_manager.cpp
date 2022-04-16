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
    for (auto &it : feature)
    {   
        it.used_num = it.feature_per_frame.size();
        if (it.used_num >= 4)
        {
            cnt++;
        }
    }
    return cnt;
}

bool FeatureManager::addFeatureCheckParallax(int frameCount, const FeatureMap &featureMap, double td)
{
    ROS_DEBUG("input feature: %d", (int)featureMap.size());
    ROS_DEBUG("num of feature: %d", getFeatureCount());

    double parallax_sum = 0;
    int parallax_num = 0;
    last_track_num = 0;
    new_feature_num = 0;
    long_track_num = 0;

    for(auto &id_pts : featureMap)
    {   // 特征点在每帧中的信息，归一化平面坐标、像素坐标、归一化平面上的速度
        FeaturePerFrame f_per_fra(id_pts.second[0].second, td);
        // 输入右目的信息
        f_per_fra.rightObservation(id_pts.second[1].second);

        int feature_id = id_pts.first;
        // 自定义查找函数，返回在feature中的迭代器
        auto it = find_if(feature.begin(), feature.end(), 
                            [feature_id](const FeaturePerId &it){
                                return it.feature_id == feature_id;
                            });
        // 找不到，说明这个点还没记录在 feature 里面，现在把它添加进去
        if (it == feature.end())
        {
            feature.push_back(FeaturePerId(feature_id, frameCount));
            feature.back().feature_per_frame.push_back(f_per_fra);
            new_feature_num++;
        }
        else if (it->feature_id == feature_id)
        {   // 记录追踪到的特征点在各帧中的特征信息
            it->feature_per_frame.push_back(f_per_fra);
            last_track_num++;
            if( it-> feature_per_frame.size() >= 4)
                long_track_num++;   // 追踪次数超过四次，说明该点长时间成功追踪
        }
    }
    if (frameCount < 2 || last_track_num < 20 || long_track_num < 40 || new_feature_num > 0.5 * last_track_num)
        return true; // 说明当前帧是新的关键帧，直接返回

    for (auto &it_per_id : feature)
    {
        if (it_per_id.start_frame <= frameCount - 2 &&
            it_per_id.start_frame + int(it_per_id.feature_per_frame.size()) - 1 >= frameCount - 1)
        {
            parallax_sum += compensatedParallax2(it_per_id, frameCount);
            parallax_num++;
        }
    }
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
