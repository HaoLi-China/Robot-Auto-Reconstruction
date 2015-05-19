#ifndef AUTO_RECONSTRUCTION_H
#define AUTO_RECONSTRUCTION_H

#include <ros/ros.h>
#include <robot_auto_reconstruction/robot_control.h>
#include <math.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "std_msgs/String.h"
#include <sstream>

#include "up_down_gripper.h"

#ifndef PI
#define PI 3.1415926
#endif

//init robot pose
void init_robot_pose(simple_robot_control::Robot &robot, double pos_left_arm[], double pos_right_arm[], tf::Vector3 &head_focus, float torso_up);

//init left arm pose
void init_left_arm_pose(simple_robot_control::Robot &robot, double pos_left_arm[]);

//init right arm pose
void init_right_arm_pose(simple_robot_control::Robot &robot, double pos_right_arm[]);

//make left scanner up and down
void up_down_left_scanner(PR2MoveArmJoint &move_arm_joint, float down_value, float up_value);

//make right scanner up and down
void up_down_right_scanner(PR2MoveArmJoint &move_arm_joint, float down_value, float up_value);

//r_arm pick up kinect
void r_pick_up_kinect(simple_robot_control::Arm& arm_r, simple_robot_control::Gripper& gripper_r, simple_robot_control::Base& base);

//r_arm put down kinect
void r_put_down_kinect(simple_robot_control::Arm& arm_r, simple_robot_control::Gripper& gripper_r, simple_robot_control::Base& base);

//l_arm pick up kinect
void l_pick_up_kinect(simple_robot_control::Arm& arm_l, simple_robot_control::Gripper& gripper_l, simple_robot_control::Base& base);

//l_arm put down kinect
void l_put_down_kinect(simple_robot_control::Arm& arm_l, simple_robot_control::Gripper& gripper_l, simple_robot_control::Base& base);

//left arm push object
void l_push_object(simple_robot_control::Arm& arm_l, tf::Vector3& position, tf::Vector3& direction);

//right arm push object
void r_push_object(simple_robot_control::Arm& arm_r, tf::Vector3& position, tf::Vector3& direction);

//left arm take back
void l_take_back(simple_robot_control::Arm& arm_l, tf::Vector3& position, tf::Vector3& direction);

//right arm take back
void r_take_back(simple_robot_control::Arm& arm_r, tf::Vector3& position, tf::Vector3& direction);

//set head pose
void set_head_pose(simple_robot_control::Head& head, tf::Vector3 &head_focus);

//get left gripper touch point and direction
void get_l_touch_point_and_dir(ros::NodeHandle &n, const tf::Vector3 &position_kinect, const tf::Vector3 &dir_kinect, tf::Vector3 &position_base, tf::Vector3 &dir_base);

//get right gripper touch point and direction
void get_r_touch_point_and_dir(ros::NodeHandle &n, const tf::Vector3 &position_kinect, const tf::Vector3 &dir_kinect, tf::Vector3 &position_base, tf::Vector3 &dir_base);

//just for test
void test_calibration_result(ros::NodeHandle &n, const tf::Vector3 &position_kinect, const tf::Vector3 &dir_kinect);
#endif
