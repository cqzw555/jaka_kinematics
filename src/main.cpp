#include <math.h>
#include <vector>
#include <stdio.h>
#include "jaka_kinematics/jaka_kinematics.h"


int main()
{
    // double matrix[] = {1, 0, 0, 0.114195, 0, 1, 0, 0.043295, 0, 0, 1, 0.549373, 0, 0, 0, 1};
    // double matrix[] = {1, 0, 0, 0.114195, 0, 0.2588, -0.965927, 0.043295, 0, 0.965927, 0.2588, 0.549373, 0, 0, 0, 1};
    double matrix[16] = {0};
    double trans[3] = {0.360, 0.05, 0.6};
    double rpy[3] = {PI / 3, PI / 9, -174.066 * PI / 180};
    // 对应的角度是 394.395 107737 -126.566 198.829 325.605 90
    // 对应弧度是 6.883 1.8803 -2.2089 3.4702 5.6828 1.5707
    double rot[9] = {0};
    jaka_kinematics kine;
    kine.from_rpy(rot, rpy);
    kine.to_mat44(matrix, trans, rot);
    for (int i = 0; i < 16; i++)
        printf("%.6f ", matrix[i]);
    printf("\n");
    double solution[48] = {0};

    auto num = kine.inverse(matrix, solution, 0);
    printf("%d\n", num);
    printf("找出来的解:\n");
    for (int i = 0; i < num; i++)
    {
        for (int j = 0; j < 6; j++)
            printf("%.6f ", solution[i * 6 + j] * 180 / PI);
        printf("\n");
    }
    printf("正运动学结果:\n");
    for (int i = 0; i < num; i++)
    {
        double T[16] = {-143.410, 86.157, 108.558, 19.913, 124.225, 178.431};
        for (int j = 0; j < 16; j++)
            T[j] = T[j] * PI / 180;
        kine.forward(&solution[i * 6], T);
        for (int j = 0; j < 16; j++)
        {
            printf("%.6f ", T[j]);
        }
        printf("\n");
    }

    return 0;
}