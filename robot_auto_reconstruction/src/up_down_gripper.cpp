#include "up_down_gripper.h"

PR2MoveArmJoint::PR2MoveArmJoint()
{
    // tell the action client that we want to spin a thread by default (true)
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up (while)
    while (!traj_client_->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Hi, i'm Waiting for the joint_trajectory_action server");
    }
}

// clean up the action client
PR2MoveArmJoint::~PR2MoveArmJoint()
{
    delete traj_client_;
}

// send the command to start a given trajectory
void PR2MoveArmJoint::startTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal)
{
    // when to start the trajectory: 1s from now
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
    traj_client_->sendGoal(goal);
}

void PR2MoveArmJoint::get_current_joint_angles(double cur_joint_angles[7])
{
    // get a single message from the topic "r_arm_controller/state"
    pr2_controllers_msgs::JointTrajectoryControllerStateConstPtr state_msg = ros::topic::waitForMessage<pr2_controllers_msgs::JointTrajectoryControllerState> ("r_arm_controller/state");

    // extract the joint angles
    for (int i = 0; i < 7; ++i)
    {
        cur_joint_angles[i] = state_msg->actual.positions[i];
    }
}

// up and down the scanner
pr2_controllers_msgs::JointTrajectoryGoal PR2MoveArmJoint::armScanTrajectory(int flag, float down_value, float up_value)
{
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    if(flag==0){
        goal.trajectory.joint_names.push_back("l_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("l_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("l_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("l_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("l_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("l_wrist_roll_joint");
        goal.trajectory.points.resize(2);

        double current_angles[7];
        get_current_joint_angles(current_angles);

        // double initalRightWristFlex = current_angles[5];

        // First trajectory point: up
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(7);
        goal.trajectory.points[ind].positions[0] = current_angles[0];
        goal.trajectory.points[ind].positions[1] = current_angles[1];
        goal.trajectory.points[ind].positions[2] = current_angles[2];
        goal.trajectory.points[ind].positions[3] = current_angles[3];
        goal.trajectory.points[ind].positions[4] = current_angles[4];
        goal.trajectory.points[ind].positions[5] = up_value;  // down
        //goal.trajectory.points[ind].positions[5] = -1.697;  // down
        //goal.trajectory.points[ind].positions[5] = -1.7;  // down
        //goal.trajectory.points[ind].positions[5] = -2.0;  // down
        //goal.trajectory.points[ind].positions[5] = -2.05;  // down
        goal.trajectory.points[ind].positions[6] = current_angles[6];
        goal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // goal.trajectory.points[ind].velocities[5] = 0.01;
        // goal.trajectory.points[ind].velocities[6] = 0.0;
        //goal.trajectory.points[ind].time_from_start = ros::Duration(8);   // 5 is great
        goal.trajectory.points[ind].time_from_start = ros::Duration(3);   // 5 is great


        // Second traj point: down
        ind += 1;
        goal.trajectory.points[ind].positions.resize(7);
        goal.trajectory.points[ind].positions[0] = current_angles[0];
        goal.trajectory.points[ind].positions[1] = current_angles[1];
        goal.trajectory.points[ind].positions[2] = current_angles[2];
        goal.trajectory.points[ind].positions[3] = current_angles[3];
        goal.trajectory.points[ind].positions[4] = current_angles[4];
        goal.trajectory.points[ind].positions[5] = down_value;  // up
        //goal.trajectory.points[ind].positions[5] = -1.547;  // up
        //goal.trajectory.points[ind].positions[5] = -1.2;  // up
        //goal.trajectory.points[ind].positions[5] = -1.6;  // up
        // goal.trajectory.points[ind].positions[5] = -1.95;  // up
        goal.trajectory.points[ind].positions[6] = current_angles[6];
        goal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        //goal.trajectory.points[ind].velocities[5] = 0.01;
        //goal.trajectory.points[ind].velocities[6] = 0.0;
        //goal.trajectory.points[ind].time_from_start = ros::Duration(16);  // 10 is great
        goal.trajectory.points[ind].time_from_start = ros::Duration(6);  // 10 is great
    }
    else if(flag==1){
        goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
        goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
        goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
        goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
        goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
        goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
        goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
        goal.trajectory.points.resize(2);

        double current_angles[7];
        get_current_joint_angles(current_angles);

        // double initalRightWristFlex = current_angles[5];

        // First trajectory point: up
        int ind = 0;
        goal.trajectory.points[ind].positions.resize(7);
        goal.trajectory.points[ind].positions[0] = current_angles[0];
        goal.trajectory.points[ind].positions[1] = current_angles[1];
        goal.trajectory.points[ind].positions[2] = current_angles[2];
        goal.trajectory.points[ind].positions[3] = current_angles[3];
        goal.trajectory.points[ind].positions[4] = current_angles[4];
        goal.trajectory.points[ind].positions[5] = up_value;  // down
        //goal.trajectory.points[ind].positions[5] = -1.697;  // down
        //goal.trajectory.points[ind].positions[5] = -1.7;  // down
        //goal.trajectory.points[ind].positions[5] = -2.0;  // down
        //goal.trajectory.points[ind].positions[5] = -2.05;  // down
        goal.trajectory.points[ind].positions[6] = current_angles[6];
        goal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        // goal.trajectory.points[ind].velocities[5] = 0.01;
        // goal.trajectory.points[ind].velocities[6] = 0.0;
        //goal.trajectory.points[ind].time_from_start = ros::Duration(8);   // 5 is great
        goal.trajectory.points[ind].time_from_start = ros::Duration(4);   // 5 is great


        // Second traj point: down
        ind += 1;
        goal.trajectory.points[ind].positions.resize(7);
        goal.trajectory.points[ind].positions[0] = current_angles[0];
        goal.trajectory.points[ind].positions[1] = current_angles[1];
        goal.trajectory.points[ind].positions[2] = current_angles[2];
        goal.trajectory.points[ind].positions[3] = current_angles[3];
        goal.trajectory.points[ind].positions[4] = current_angles[4];
        goal.trajectory.points[ind].positions[5] = down_value;  // up
        //goal.trajectory.points[ind].positions[5] = -1.547;  // up
        //goal.trajectory.points[ind].positions[5] = -1.2;  // up
        //goal.trajectory.points[ind].positions[5] = -1.6;  // up
        // goal.trajectory.points[ind].positions[5] = -1.95;  // up
        goal.trajectory.points[ind].positions[6] = current_angles[6];
        goal.trajectory.points[ind].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[ind].velocities[j] = 0.0;
        }
        //goal.trajectory.points[ind].velocities[5] = 0.01;
        //goal.trajectory.points[ind].velocities[6] = 0.0;
        //goal.trajectory.points[ind].time_from_start = ros::Duration(16);  // 10 is great
        goal.trajectory.points[ind].time_from_start = ros::Duration(6);  // 10 is great
    }

    return goal;
}

pr2_controllers_msgs::JointTrajectoryGoal PR2MoveArmJoint::armBacktoOrigin()
{
    pr2_controllers_msgs::JointTrajectoryGoal goal;

    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");
    goal.trajectory.points.resize(1);

    // original position
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].positions[0] = 0.564;
    goal.trajectory.points[0].positions[1] = 0.365;
    goal.trajectory.points[0].positions[2] = 0.312;
    goal.trajectory.points[0].positions[3] = -1.531;
    goal.trajectory.points[0].positions[4] = 2.646;
    goal.trajectory.points[0].positions[5] = -1.068;
    goal.trajectory.points[0].positions[6] = 3.076; // new point: 0.564 0.365 0.312 -1.531 2.646 -1.068 3.076
    goal.trajectory.points[0].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
        goal.trajectory.points[0].velocities[j] = 0.0;
    }
    goal.trajectory.points[0].time_from_start = ros::Duration(12);

    return goal;
}

actionlib::SimpleClientGoalState PR2MoveArmJoint::getState()
{
    return traj_client_->getState();
}
