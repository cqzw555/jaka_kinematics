#ifndef JAKA_KINEMATICS_H
#define JAKA_KINEMATICS_H
#include <math.h>
#define JAKA_KINEMATICS_DEBUG
#ifdef JAKA_KINEMATICS_DEBUG
#include <stdio.h>
#endif

#define PI M_PI
namespace jaka_kinematics
{
    const double ZERO_THRESH = 1e-8;

    // @param q       The 6 joint values
    // @param T       The 4x4 end effector pose in row-major ordering
    // @return        None
    void forward(double *q, double *T);

    // @param T       The 4x4 end effector pose in row-major ordering
    // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
    // @param q6_des  An optional parameter which designates what the q6 value should take
    //                in case of an infinite solution on that joint.
    // @return        Number of solutions found (maximum of 8)
    int inverse(double *T, double *q_sols, double q6_des = 0.0);

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