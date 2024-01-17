#ifndef JAKA_KINEMATICS_H
#define JAKA_KINEMATICS_H
#include <math.h>
#include <vector>
#define JAKA_KINEMATICS_DEBUG
#ifdef JAKA_KINEMATICS_DEBUG
#include <stdio.h>
#endif

#define PI M_PI
class jaka_kinematics
{
    double d1 = 0.1211;
    double a2 = 0.429733;
    double a3 = 0.368162;
    
    double d4 = -0.114195;
    double d5 = 0.113416;
    double d6 = 0.107119;

    double joint_min[6] = {-1.5 * PI, -85 * PI / 180, -175 * PI / 180, -1.5 * PI, -1.5 * PI};
    double joint_max[6] = {1.5 * PI, 265 * PI / 180, 175 * PI / 180, 1.5 * PI, 1.5 * PI};
    const double ZERO_THRESH = 1e-8;
public:
    jaka_kinematics(const double* dh_parameter);
    jaka_kinematics();
    jaka_kinematics(const std::vector<double> para);
    
    // @param q       The 6 joint values
    // @param T       The 4x4 end effector pose in row-major ordering
    // @return        None
    void forward(const double *q, double *T);
    
    std::vector<double> forward_vector(const std::vector<double> joints){
        std::vector<double> res(16,0);
        forward(&joints[0],&res[0]);
        return res;
    }

    // @param T       The 4x4 end effector pose in row-major ordering
    // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
    // @param q6_des  An optional parameter which designates what the q6 value should take
    //                in case of an infinite solution on that joint.
    // @return        Number of solutions found (maximum of 8)
    int inverse(const double *T, double *q_sols, double q6_des = 0.0);
    std::vector<double> inverse_vector(const std::vector<double> matrix,const double q6_des){
        std::vector<double> res(6,0);
        inverse(&matrix[0],&res[0],q6_des);
        return res;
    }
    // @param T       The 4x4 end effector pose in row-major ordering
    // @param rot     The 3x3 rotate matrix that represents the pose
    // @param trans   The 3x1 vector that represents a position
    // @return        None
    void to_mat44(double *T, double *trans, double *rot);

    // @param T       The 4x4 end effector pose in row-major ordering
    // @param trans   The 3x1 vector that represents a position
    // @param rot     The 3x3 rotate matrix that represents the pose
    // @return        None
    void from_mat44(double *T, double *trans, double *rot);

    // @param rpy    Euler angles
    // @param rot    the 3x3 rotate matrix that represents the pose
    // @return       None
    void from_rpy(double *rpy, double *rot);

    // @param rot    the 3x3 rotate matrix that represents the pose
    // @param rpy    Euler angles
    // @return       None
    void to_rpy(double *rot, double *rpy);

    // Convert the angle between the given min and max
    // @param T       The angle to be entered
    // @param min     The min value to be entered
    // @param max     The max value to be entered
    // @return        The converted angle value
    double rotate(double t, double min, double max);

    // Determine the positive or negative of the giving float
    // @param T       double
    // @return        1 if positive else -1
    int SIGN(double x);
};
#endif