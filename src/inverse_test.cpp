// 逆运动学测试
#include "jaka_kinematics/jaka_kinematics.h"
#include <fstream>
#include <stdio.h>
#include <assert.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <vector>
#include <numeric>

using namespace std;
using namespace boost;
// using namespace jaka_kinematics;

// 统计位置误差和欧拉角的误差

void stats(vector<double> &data)
{
    double sum = accumulate(data.begin(), data.end(), 0.0);
    double accum = 0.0, _max = -1, _min = 999;

    double mean = sum / data.size();
    for_each(data.begin(), data.end(), [&](const double a)
             {
        accum += pow(a-mean,2);
        _min = min(a,_min);
        _max = max(a,_max); });

    double variance = accum / data.size();
    double stdev = sqrt(variance);
    printf("mean:%.4f stdev:%.4f min:%.4f max:%.4f\n", mean, stdev, _min, _max);
}

inline double angle_diff(double a, double b)
{
    return acos(cos(a) * cos(b) + sin(b) * sin(a));
}

int solution_check(double *soultion, double *correct, int solution_num, double t)
{
    for (int i = 0; i < solution_num; i++)
    {
        if (angle_diff(soultion[i * 6 + 0], correct[0]) < t &&
            angle_diff(soultion[i * 6 + 1], correct[1]) < t &&
            angle_diff(soultion[i * 6 + 2], correct[2]) < t &&
            angle_diff(soultion[i * 6 + 3], correct[3]) < t &&
            angle_diff(soultion[i * 6 + 4], correct[4]) < t &&
            angle_diff(soultion[i * 6 + 5], correct[5]) < t)
            return i;
    }
    return -1;
}

int main(int argc, char *argv[])
{
    assert(argc > 1);
    ifstream f1, f2;
    string joint_file = argv[2], line, pose_file = argv[1];
    vector<string> vStr;
    vector<double> x_error, y_error, z_error, rx_error, ry_error, rz_error;

    double T[16] = {0}, rpy[3] = {0}, joint[6] = {0}, trans[3], rot[9], pose[6], solutions[50] = {0};

    f1.open(joint_file.c_str());
    f2.open(pose_file.c_str());
    int wrong_num = 0, line_num = 0, n = 0;
    double thre = 0.01;
    while (getline(f2, line))
    {
        line_num++;
        split(vStr, line, is_any_of(" "), token_compress_on);
        assert(vStr.size() >= 6);
        for (int i = 0; i < 3; i++)
            trans[i] = atof(vStr[i].c_str());
        for (int i = 3; i < 6; i++)
            rpy[i - 3] = atof(vStr[i].c_str());

        jaka_kinematics::from_rpy(rpy, rot);
        jaka_kinematics::to_mat44(T, trans, rot);
        int num = jaka_kinematics::inverse(T, solutions);

        assert(getline(f1, line));
        if (num == 0)
        {
            n++;
            printf("逆解失败，当前位姿(x,y,z,rx,ry,rz):\n");
            for (int i = 0; i < 3; i++)
            printf("%.4f ",trans[i]);
            for (int i = 0; i < 3; i++)
            printf("%.4f ",rpy[i]);
            printf("\n***************************************************\n");
            continue;
        }
        split(vStr, line, is_any_of(" "), token_compress_on);
        assert(vStr.size() >= 6);
        for (int i = 0; i < 6; i++)
            joint[i] = atof(vStr[i].c_str());
        int i = solution_check(solutions, joint, num, thre);
        if (i == -1)
        {
            wrong_num++;
            printf("误差%.4f弧度以外的错误结果\n", thre);
            printf("逆解结果：\n");
            for (int j = 0; j < num; j++)
            {
                for (int k = 0; k < 6; k++)
                    printf("%.5f ", solutions[j * 6 + k]);
                printf("\n");
            }

            printf("正确结果：\n");
            for (int k = 0; k < 6; k++)
                printf("%.5f ", joint[k]);
            printf("\n-----------------------------------------------------\n");
        }

    }
    printf("有%d / %d个逆解失败\n", n, line_num);
    printf("有%d / %d个没有找到正确结果,角度误差%.4f弧度以上\n", wrong_num, line_num, thre);

    f1.close();
    f2.close();
}