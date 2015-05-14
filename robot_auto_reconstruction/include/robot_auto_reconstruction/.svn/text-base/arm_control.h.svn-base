/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Robert Bosch LLC.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Robert Bosch nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *********************************************************************/

/*
 * arm_control.h
 *
 *  Created on: Jun 30, 2010
 *      Author: christian
 */

#ifndef ARM_CONTROL_H_
#define ARM_CONTROL_H_

#include <string>
#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <arm_navigation_msgs/MoveArmAction.h>
#include <kinematics_msgs/GetPositionIK.h>







namespace simple_robot_control{


using ::std::string;
using ::std::vector;


class Arm{
private:
		const static double wrist_speed_ = 2.0; //2 seconds per revolution
		const static double gripperlength = 0.18;



	  vector<string> joint_names;
	  string armside_str;
	  string group_name;
	  actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>* move_arm_client_;
	  actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > traj_client_;

	  ros::ServiceClient ik_service_client_;
	  ros::NodeHandle nh_;


	  vector<double> current_joint_angles_;

public:


	  /**
	   * Constructor: arm_side should be either 'r' or 'l'; collision checking enables ompl planner
	   */
	  Arm(char arm_side, bool collision_checking = true);


	  ~Arm();

	  ////////////
	  //joint angle control
	  ///////////

	  /**
	   * moves arm to joint angle position. positions should contain a multiple of the 7 arm joint positions to create a trajectory
	   */
	  bool goToJointPos (const vector<double>& positions , double max_time = 3.0, bool wait = true );


	  /**
	   * uses ompl path planner and moves arm to  joint position. positions should contain a multiple of the 7 arm joint positions to create a trajectory.
	   */
	  bool goToJointPosWithCollisionChecking (const vector<double>& positions, double max_time = 3.0, bool wait = true );


	  /**
	   * Rotates wrist by angle in radians. speed is the amount of time that it would take for one revolution.
	   */
	  bool rotateWrist(double radians, double wrist_speed = wrist_speed_ ,bool wait = true);



	  ///////////////
	  //euclidian position control
	  //////////////
	  /**
	   * get IK from kinematics server
	   */
	  bool getIK(const geometry_msgs::PoseStamped& pose,  vector<double>& joint_angles, vector<double>* ik_seed_pos = NULL);


	  /**
	   * get joint angles
	   */
	  vector<double> getCurrentJointAngles();



	  enum approach_direction_t { FRONTAL, FROM_BELOW, FROM_ABOVE, FROM_RIGHT_SIDEWAYS, FROM_RIGHT_UPRIGHT, FROM_LEFT_UPRIGHT, FROM_LEFT_SIDEWAYS};
	  /**
	   * gets IK and moves gripper_tool_frame to pose specified in postion and the orientation of the approach direction relative to base coordinate frame
	   */
	  bool moveGripperToPosition(const tf::Vector3& position,  string frame_id, approach_direction_t approach = FRONTAL,  double max_time =5.0, bool wait=true, vector<double>* ik_seed_pos = 0  );


	  /**
	   * gets IK and moves gripper_tool_frame to pose specified in postion and the orientation of the approach direction relative to base coordinate frame
	   */
	  bool moveGrippertoPositionWithCollisionChecking(const tf::Vector3& position,  string frame_id, approach_direction_t approach = FRONTAL,  double max_time =5.0, bool wait=true, std::string planner = "ompl"  );


	  /**
	   * gets IK and moves gripper_tool_frame to pose specified in pose. The orientation can be overridden by above enums.
	   */
	  bool moveGrippertoPose(const tf::StampedTransform& pose,   double max_time =5.0, bool wait=true, vector<double>* ik_seed_pos = 0  );

	  /**
	   * uses ompl path planner and moves gripper_tool_frame to pose specified in pose. The orientation can be overridden by above enums.
	   */
	  bool moveGrippertoPoseWithCollisionChecking(const tf::StampedTransform& pose,  double max_time =5.0, bool wait=true, std::string planner = "ompl" );


	  /**
	   * uses ompl planner to calculate path and moves wrist roll link to pose specified and keeps the set orientations on the motion path within the tolerance (angle in radians)
	   */
	  bool moveGrippertoPoseWithOrientationConstraints(const tf::StampedTransform& tf, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time =5.0, bool wait=true, double tolerance = 0.2);






	  /**
	   * gets IK and moves wrist roll link to pose specified in tf
	   */
	  bool moveWristRollLinktoPose(const tf::StampedTransform& pose,  double max_time =5.0, bool wait=true, vector<double>* ik_seed_pos = 0  );

	  /**
	   * gets IK and moves wrist roll link to pose specified in pose
	   */
	  bool moveWristRollLinktoPose(const geometry_msgs::PoseStamped& pose,  double max_time =5.0, bool wait=true, vector<double>* ik_seed_pos = 0 );

	  /**
	   * uses ompl planner to calculate path and moves wrist roll link to pose specified in pose
	   */
	  bool moveWristRollLinktoPoseWithCollisionChecking(const geometry_msgs::PoseStamped& pose,  double max_time =5.0, bool wait=true, std::string planner = "ompl"  );


	  /**
	   * uses ompl planner to calculate path and moves wrist roll link to pose specified
	   */
	  bool moveWristRollLinktoPoseWithCollisionChecking(const tf::StampedTransform& pose,  double max_time =5.0, bool wait=true, std::string planner = "ompl"  );


	  /**
	   * uses ompl planner to calculate path and moves wrist roll link to pose specified and keeps the set orientations on the motion path within the tolerance (angle in radians)
	   */
	  bool moveWristRollLinktoPoseWithOrientationConstraints(const tf::StampedTransform& pose, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time =5.0, bool wait=true, double tolerance = 0.2);


	  /**
	   * uses ompl planner to calculate path and moves wrist roll link to pose specified and keeps the set orientations on the motion path within the tolerance (angle in radians)
	   */
	  bool moveWristRollLinktoPoseWithOrientationConstraints(const geometry_msgs::PoseStamped& pose, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time =5.0, bool wait=true, double tolerance = 0.2 );


	  /**
	   * moves arm to joint position. positions should contain a multiple of the 7 arm joint positions to create a trajectory
	   */
	  bool goToJointPos (const double* positions , int num_positions, double max_time = 3.0, bool wait = true );

	  /**
	   * sets current_joint_pos_ to current positions of the robot arm
	   */
	  bool updateJointStatePos();


	  /**
	   * Checks weather current arm position are the same as in pos_vec (wit some epsilon).
	   */
	  bool isAtPos(const std::vector<double>& pos_vec);

	  /**
	   * Move arm into tuck position.
	   */
	  bool tuck();

	  /**
	   * Move arm into stretched out position.
	   */
	  bool stretch();




private:
	  tf::StampedTransform gripperToWrist(const tf::StampedTransform& pose);
	  tf::StampedTransform wristToGripper(const tf::StampedTransform& pose);
	  tf::StampedTransform makePose(const tf::Vector3& position, string frame_id, approach_direction_t approach);





};
}

#endif /* ARM_CONTROL_H_ */
