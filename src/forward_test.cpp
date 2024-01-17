// 前向运动学测试
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
int main(int argc, char *argv[])
{
    assert(argc > 1);
    ifstream f1, f2;
    string joint_file = argv[1], line, pose_file = argv[2];
    vector<string> vStr;
    vector<double> x_error, y_error, z_error, rx_error, ry_error, rz_error;

    double T[16] = {0}, rpy[3] = {0}, joint[6] = {0}, trans[3], rot[9], pose[6];

    f1.open(joint_file.c_str());
    f2.open(pose_file.c_str());
    jaka_kinematics kine;
    while (getline(f1, line))
    {
        split(vStr, line, is_any_of(" "), token_compress_on);
        assert(vStr.size() >= 6);
        for (int i = 0; i < 6; i++)
            joint[i] = atof(vStr[i].c_str());
        kine.forward(joint, T);
        kine.from_mat44(T, trans, rot);
        kine.to_rpy(rot, rpy);

        assert(getline(f2, line));
        // printf("%s\n",line.c_str());
        split(vStr, line, is_any_of(" "), token_compress_on);
        assert(vStr.size() >= 6);
        for (int i = 0; i < 6; i++)
            pose[i] = atof(vStr[i].c_str());
        x_error.emplace_back(fabs(trans[0]-pose[0]));
        y_error.emplace_back(fabs(trans[1]-pose[1]));
        z_error.emplace_back(fabs(trans[2]-pose[2]));

        rx_error.emplace_back(fabs(angle_diff(rpy[0],pose[3])));
        ry_error.emplace_back(fabs(angle_diff(rpy[1],pose[4])));
        rz_error.emplace_back(fabs(angle_diff(rpy[2],pose[5])));

        if (fabs(rpy[0] - pose[3]) > 0.1 || fabs(rpy[1] - pose[4]) > 0.1 || fabs(rpy[2] - pose[5]) > 0.1)
        {
            for (int i = 0; i < 3; i++)
                printf("%.5f ", trans[i]);
            for (int i = 0; i < 3; i++)
                printf("%.5f ", rpy[i] * 180 / M_PI);
            printf("\n");
            for (int i = 0; i < 3; i++)
                printf("%.5f ", pose[i]);
            for (int i = 3; i < 6; i++)
                printf("%.5f ", pose[i] * 180 / M_PI);
            printf("\n---------------------------------------------\n");
        }
    }
    {
        printf("x:\n"); stats(x_error);
        printf("y:\n"); stats(y_error);
        printf("z:\n"); stats(z_error);
        printf("rx:\n"); stats(rx_error);
        printf("ry:\n"); stats(ry_error);
        printf("rz:\n"); stats(rz_error);
    }
    f1.close();
    f2.close();
}