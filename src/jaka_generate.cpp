// #include "jaka_kinematics/jaka_kinematics.h"
#include "ros/ros.h"
#include "jaka_planner/JAKAZuRobot.h"
#include "jaka_planner/jkerr.h"
#include "jaka_planner/jktypes.h"
#include <sensor_msgs/JointState.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <string>
#include <map>
#include "std_srvs/Empty.h"
#include "std_srvs/SetBool.h"
#include "std_msgs/Empty.h"
#include <thread>
#include <cstdlib>
#include <fstream>

// using namespace jaka_kinematics;
using namespace std;
int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "moveit_server");
    ros::NodeHandle nh;
    string default_ip = "192.168.0.120";
    string default_model = "zu5";
    // string robot_ip = nh.param("ip", default_ip);
    // string robot_model = nh.param("model", default_model);
    JAKAZuRobot robot;
    robot.login_in(default_ip.c_str());
    // robot.set_status_data_update_time_interval(100);
    ros::Rate rate(125);
    robot.servo_move_enable(false);
    ros::Duration(0.5).sleep();
    // Set filter parameter
    robot.servo_move_use_joint_LPF(0.5);
    robot.power_on();
    robot.enable_robot();

    CartesianPose pose;
    JointValue joint = {0};
    JointValue ref_joint = {-60 * M_PI / 180, 52 * M_PI / 180, 139 * M_PI / 180, -11 * M_PI / 180, 60 * M_PI / 180, 60 * M_PI / 180};
    int i = 0;
    ofstream f1, f2;
    f1.open("joint.txt");
    f2.open("pose.txt");
    int nums = 10086;
    while (ros::ok())
    {
        if (i >= nums)
            break;
        int idx = (int)(rand() % 6);
        joint.jVal[idx] = (rand()*1.0/RAND_MAX) * 2 * M_PI;
        auto res = robot.kine_forward(&joint, &pose);
        f2 << setprecision(5);
        f1 << setprecision(5);
        
        if (res != ERR_SUCC)
        {
            continue;
        }
        printf("\rgenerate the %d / %d case",i,nums);
        for (int j = 0; j < 6; j++)
            f1 << joint.jVal[j] << " ";
        f1 << endl;

        double *p = (double*)&pose;
        for(int j=0;j<3;j++) p[j] /= 1000;
        for (int j = 0; j < 6; j++)
            f2 << p[j] << " ";
        f2 << endl;
        i++;
    }
    printf("\ndone\n");

    // robot.disable_robot();
    // robot.power_off();
}
