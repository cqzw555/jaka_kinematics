// 前向运动学测试
#include "jaka_kinematics/jaka_kinematics.h"
#include <fstream>
#include <stdio.h>
#include <assert.h>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <vector>

using namespace std;
using namespace boost;
// using namespace jaka_kinematics;

int main(int argc, char *argv[])
{
    assert(argc > 1);
    ifstream f1,f2;
    string joint_file = argv[1], line,pose_file = argv[2];
    vector<string> vStr;
    double T[16] = {0}, rpy[3] = {0}, joint[6] = {0},trans[3],rot[9],pose[6];

    f1.open(joint_file.c_str());
    f2.open(pose_file.c_str());
    while (getline(f1, line))
    {
        split(vStr, line, is_any_of(" "), token_compress_on);
        assert(vStr.size() >= 6);
        for (int i = 0; i < 6; i++)
            joint[i] = atof(vStr[i].c_str());
        jaka_kinematics::forward(joint, T);
        jaka_kinematics::from_mat44(T,trans,rot);
        jaka_kinematics::to_rpy(rot,rpy);
        for (int i = 0; i < 3; i++)
            printf("%.5f ", trans[i]);
        for (int i = 0; i < 3; i++)
            printf("%.5f ", rpy[i]*180/M_PI);
        printf("\n");
        
        assert(getline(f2,line));
        // printf("%s\n",line.c_str());
        split(vStr, line, is_any_of(" "), token_compress_on);
        assert(vStr.size() >= 6);
        for (int i = 0; i < 6; i++)
            pose[i] = atof(vStr[i].c_str());
        for (int i = 0; i < 3; i++)
            printf("%.5f ", pose[i]);
        for (int i = 3; i < 6; i++)
            printf("%.5f ", pose[i]*180/M_PI);
             printf("\n");
        printf("---------------------------------------------\n");
    }
    f1.close();
}