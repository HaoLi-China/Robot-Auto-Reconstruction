#ifndef UP_DOWN_GRIPPER_H
#define UP_DOWN_GRIPPER_H

#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajClient;

class PR2MoveArmJoint
{
private:
    TrajClient* traj_client_;

public:
    PR2MoveArmJoint();

    // clean up the action client
    ~PR2MoveArmJoint();

    // send the command to start a given trajectory
    void startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal);

    void get_current_joint_angles(double cur_joint_angles[7]);

    // up and down the scanner
    pr2_controllers_msgs::JointTrajectoryGoal armScanTrajectory(int flag, float down_value, float up_value);

    pr2_controllers_msgs::JointTrajectoryGoal armBacktoOrigin();

    actionlib::SimpleClientGoalState getState();

};

#endif
