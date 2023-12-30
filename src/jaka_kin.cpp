#include "jaka_kinematics/jaka_kinematics.h"
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
#include <random>
// using namespace jaka_kinematics;
using namespace std;
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
  pose.rpy.rx = -76 * M_PI / 180;
  pose.rpy.rx = -20 * M_PI / 180;
  pose.rpy.rx = -174 * M_PI / 180;
  pose.tran.x = 54.946;
  pose.tran.y = 124.797;
  pose.tran.z = 593.577;

  JointValue joint;
  JointValue ref_joint = {-60 * M_PI / 180, 52 * M_PI / 180, 139 * M_PI / 180, -11 * M_PI / 180, 60 * M_PI / 180, 60 * M_PI / 180};
  while (ros::ok())
  {
    printf("\r running!!!\n");
    auto sdk_res = robot.kine_inverse(&ref_joint, &pose, &joint);

    if (sdk_res != ERR_SUCC)
    {
      ROS_INFO("don not find the solution from jaka api");
      printf("sdk_res is ERR_KINE_INVERSE_ERR:%d \n", sdk_res == ERR_KINE_INVERSE_ERR);
      pose.tran.y += (rand()*1.0/RAND_MAX)*20.0-10;
      // continue;
    }

    double rpy[3] = {pose.rpy.rx, pose.rpy.ry, pose.rpy.rz};
    double tran[3] = {pose.tran.x / 1000, pose.tran.y / 1000, pose.tran.z / 1000};
    double rot[9] = {0};
    double T[16] = {0};
    double solution[48] = {0};
    jaka_kinematics::from_rpy(rpy, rot);
    jaka_kinematics::to_mat44(T, tran, rot);
    // for (int i = 0; i < 16; i++)
    //   printf("%.5f ", T[i]);
    // printf("\n");

    int num = jaka_kinematics::inverse(T, solution);
    if (num == 0)
    {
      ROS_INFO("don not find the solution from jaka_kinematics");
      pose.tran.x += (rand()*1.0/RAND_MAX)*20.0-10;;
      continue;
    }
    else
    {
      // ROS_INFO("find the solution from jaka_kinematics");
      int idx = solution_check(solution, (double *)&joint, num, 1e-3);
      printf("%d / %d\n",idx,num);
      if (idx == -1)
      {
        printf("没找到正确解\n");
        printf("the inverse solution from jaka_kinematics\n");
        for (int i = 0; i < num; i++)
        {
          for (int j = 0; j < 6; j++)
            printf("%.6f ", solution[6 * i + j]);
          printf("\n");
        }
        printf("the inverse solution from jaka api\n");

        for (int i = 0; i < 6; i++)
          printf("%.6f ", joint.jVal[i]);
        printf("\n");
      }
    }
    *((double*)&pose.tran + rand()%3) += (rand()*1.0/RAND_MAX)*20.0-10;;
    sleep(1);
  }

  // robot.disable_robot();
  // robot.power_off();
}
