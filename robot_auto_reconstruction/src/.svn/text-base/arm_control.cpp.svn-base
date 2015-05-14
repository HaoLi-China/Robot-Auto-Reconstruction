/*
 * arm_control.cpp
 *
 *  Created on: Jul 27, 2010
 *      Author: christian
 */

#define HALF_SQRT_TWO 0.707106781


#include <simple_robot_control/arm_control.h>
#include <tf/transform_listener.h>
#include <arm_navigation_msgs/utils.h>
#include <simple_robot_control/ReturnJointStates.h>

namespace simple_robot_control{

Arm::Arm(char arm_side, bool collision_checking): armside_str (&arm_side, 1), move_arm_client_(NULL),
		traj_client_(string(armside_str) + "_arm_controller/joint_trajectory_action", true)
{
	if (arm_side !='l' && arm_side != 'r' ) ROS_ERROR("Arm:specify l or r for arm side");

	// First, the joint names, which apply to all waypoints
	joint_names.reserve(7);
	joint_names.push_back(armside_str+"_shoulder_pan_joint");
	joint_names.push_back(armside_str +"_shoulder_lift_joint");
	joint_names.push_back(armside_str +"_upper_arm_roll_joint");
	joint_names.push_back(armside_str +"_elbow_flex_joint");
	joint_names.push_back(armside_str +"_forearm_roll_joint");
	joint_names.push_back(armside_str +"_wrist_flex_joint");
	joint_names.push_back(armside_str +"_wrist_roll_joint");

	std::string ik_service_name;
	if(arm_side == 'r'){
		ik_service_name = "pr2_right_arm_kinematics/get_ik";
	}else{
		ik_service_name = "pr2_left_arm_kinematics/get_ik";
	}

	if (!ros::service::waitForService(ik_service_name, ros::Duration(5.0))){
		ROS_ERROR("Could not find ik server %s", ik_service_name.c_str());
	}
	ik_service_client_ = nh_.serviceClient<kinematics_msgs::GetPositionIK>(ik_service_name);


	// wait for action server to come up
	if(!traj_client_.waitForServer(ros::Duration(15.0))){
		ROS_ERROR("Could not find joint_trajectory_action server %s", (armside_str + "_arm_controller/joint_trajectory_action").c_str());
	}

	if (collision_checking){
		if(arm_side == 'r'){
			group_name = "right_arm";
			move_arm_client_ = new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>("move_right_arm",true);
		}
		else if (arm_side == 'l'){
			group_name = "left_arm";
			move_arm_client_ = new actionlib::SimpleActionClient<arm_navigation_msgs::MoveArmAction>("move_left_arm",true);
		}

		else{
			ROS_ERROR("WRONG ARM NAME");
	//		ros::shutdown();
		}

		if(!move_arm_client_->waitForServer(ros::Duration(5.0))){
			ROS_INFO("could not find collision move arm client");
			move_arm_client_ = NULL;
		}
	}
	updateJointStatePos();


}

//! Clean up the action client
Arm::~Arm()
{
	delete move_arm_client_;

}



bool Arm::moveWristRollLinktoPose(const tf::StampedTransform& tf,  double max_time, bool wait, vector<double>* ik_seed_pos  ){
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

	tf::poseStampedTFToMsg(tf_pose,pose);
	return moveWristRollLinktoPose(pose, max_time, wait, ik_seed_pos);

}

bool Arm::moveWristRollLinktoPoseWithCollisionChecking(const tf::StampedTransform& tf,  double max_time, bool wait, std::string planner){
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

	tf::poseStampedTFToMsg(tf_pose,pose);
	return moveWristRollLinktoPoseWithCollisionChecking(pose, max_time, wait, planner);
}


bool Arm::goToJointPosWithCollisionChecking(const vector<double>& positions, double max_time, bool wait){

	if (!move_arm_client_){
		ROS_ERROR("collosion checking arm server has not been started");
		return false;
	}
	arm_navigation_msgs::MoveArmGoal goalB;

	goalB.motion_plan_request.group_name = group_name;
	goalB.motion_plan_request.num_planning_attempts = 1;
	goalB.motion_plan_request.allowed_planning_time = ros::Duration(max_time);

	goalB.motion_plan_request.planner_id= std::string("");
	goalB.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalB.motion_plan_request.goal_constraints.joint_constraints.resize(7);

	for (unsigned int i = 0 ; i < goalB.motion_plan_request.goal_constraints.joint_constraints.size(); ++i)
	{
	    goalB.motion_plan_request.goal_constraints.joint_constraints[i].joint_name = joint_names[i];
	    goalB.motion_plan_request.goal_constraints.joint_constraints[i].position = positions[i];
	    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_below = 0.1;
	    goalB.motion_plan_request.goal_constraints.joint_constraints[i].tolerance_above = 0.1;
	}


	bool finished_within_time = false;
	bool success = false;
	move_arm_client_->sendGoal(goalB);
	finished_within_time = move_arm_client_->waitForResult(ros::Duration(5*max_time));
	if (!finished_within_time)
	{
		move_arm_client_->cancelGoal();
		ROS_INFO("Timed out achieving  JointPos goal");
	}
	else
	{
		actionlib::SimpleClientGoalState state = move_arm_client_->getState();
	    success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
		if(success)
			ROS_INFO("Action finished: %s",state.toString().c_str());
		else
			ROS_INFO("Action failed: %s",state.toString().c_str());
	}

	return finished_within_time && success;



}

bool Arm::moveWristRollLinktoPoseWithCollisionChecking(const geometry_msgs::PoseStamped& pose,  double max_time , bool wait, std::string planner ){


	if (!move_arm_client_){
		ROS_ERROR("collision checking arm server has not been started");
		return false;
	}
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = group_name;
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("");
	if (planner == "chomp"){
		ROS_INFO("using chomp planner");
		goalA.planner_service_name = std::string("/chomp_planner_longrange/plan_path");
	}else{
		ROS_INFO("using ompl planner");
		goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	}



	goalA.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

	arm_navigation_msgs::SimplePoseConstraint desired_pose;

	desired_pose.header.frame_id = pose.header.frame_id;
	desired_pose.link_name =  armside_str + "_wrist_roll_link";
	desired_pose.pose = pose.pose;

	desired_pose.absolute_position_tolerance.x = 0.02;
	desired_pose.absolute_position_tolerance.y = 0.02;
	desired_pose.absolute_position_tolerance.z = 0.02;

	desired_pose.absolute_roll_tolerance = 0.04;
	desired_pose.absolute_pitch_tolerance = 0.04;
	desired_pose.absolute_yaw_tolerance = 0.04;

	arm_navigation_msgs::addGoalConstraintToMoveArmGoal(desired_pose,goalA);

	move_arm_client_->sendGoal(goalA);
	bool finished_before_timeout = false;
	bool success = false;

	if (wait){
		finished_before_timeout = move_arm_client_->waitForResult(ros::Duration(40.0));
		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = move_arm_client_->getState();
			success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else
			ROS_INFO("Action did not finish before the time out.");
	}
	return finished_before_timeout && success;

}

vector<double> Arm::getCurrentJointAngles(){
	updateJointStatePos();
	return current_joint_angles_;
}

bool Arm::getIK(const geometry_msgs::PoseStamped& pose,  vector<double>& joint_angles, vector<double>* ik_seed_pos){
	kinematics_msgs::GetPositionIK service_call;

	if (ik_seed_pos){
		service_call.request.ik_request.ik_seed_state.joint_state.position = *ik_seed_pos;
	}else{

		service_call.request.ik_request.ik_seed_state.joint_state.position = getCurrentJointAngles();

	}

	service_call.request.timeout = ros::Duration(5.0);
	service_call.request.ik_request.ik_seed_state.joint_state.name = joint_names;
	service_call.request.ik_request.pose_stamped = pose;
	service_call.request.ik_request.ik_link_name = armside_str +"_wrist_roll_link";
	if (!ik_service_client_.call(service_call)){
		ROS_ERROR("ik_service_call failed");
		return false;
	}

	if (service_call.response.error_code.val != 1){
		ROS_ERROR("Could not get valid IK: error code %d", service_call.response.error_code.val);
		return false;
	}

	joint_angles = service_call.response.solution.joint_state.position;
	return true;

}



bool Arm::moveWristRollLinktoPose(const geometry_msgs::PoseStamped& pose,  double max_time, bool wait, vector<double>* ik_seed_pos  ){

	vector<double> joint_angles;
	if (!getIK(pose ,joint_angles , ik_seed_pos)){
		return false;
	}

	return goToJointPos(joint_angles, max_time, wait );

}


bool Arm::goToJointPos (const double* positions , int num_positions, double max_time , bool wait ){
	std::vector<double> pos_vec (positions, positions + 7 * num_positions);
	return goToJointPos(pos_vec, max_time, wait);
}

bool Arm::goToJointPos (const vector<double>& positions , double max_time, bool wait ){

	if (positions.size() % 7 != 0){
		ROS_ERROR("you must specify 7 (or a multiple thereof) for the arm joint positions");
	}
	//our goal variable
	pr2_controllers_msgs::JointTrajectoryGoal goal;
	goal.trajectory.joint_names = joint_names;
	unsigned int num_waypoints = positions.size() / 7;
	goal.trajectory.points.resize(num_waypoints);
	vector<double>::const_iterator it = positions.begin();
	vector<double>::const_iterator it_end = positions.begin() + 7;

	for (unsigned int i=0; i <num_waypoints; i++){

		goal.trajectory.points[i].positions.insert(goal.trajectory.points[i].positions.begin(), it, it_end);
		goal.trajectory.points[i].velocities.resize(7);
		goal.trajectory.points[i].time_from_start = ros::Duration( (i+1) * max_time / num_waypoints);
		it =it_end;
		it_end+=7;
	}

	// When to start the trajectory: 1s from now
	goal.trajectory.header.stamp = ros::Time::now();// + ros::Duration(1.0);
	ROS_INFO("sending goal");
	traj_client_.sendGoal(goal);
	bool finished_before_timeout = false;
	if (wait){
		 finished_before_timeout =traj_client_.waitForResult(ros::Duration(3*max_time));
			if (finished_before_timeout)
			{
				actionlib::SimpleClientGoalState state = traj_client_.getState();
				ROS_INFO("move to joint pos Action finished: %s",state.toString().c_str());
			}
			else
				ROS_INFO("move to joint pos Action did not finish before the time out.");

	}

	return finished_before_timeout;
}

bool Arm::updateJointStatePos(){
	simple_robot_control::ReturnJointStates req;
	req.request.name = joint_names;

	if(!ros::service::waitForService("return_joint_states", ros::Duration(5.0))){
		ROS_ERROR("Arm:Could not contact return_joint_states service. Did you run joint_state_listner?");
		return false;
	}
	if (!ros::service::call("return_joint_states",req)){
		ROS_ERROR("Arm service call failed");
	}
	current_joint_angles_=  req.response.position;
	return true;

}


bool Arm::rotateWrist(double radians, double wrist_speed  ,bool wait){


	updateJointStatePos();
	vector<double> new_pos = current_joint_angles_;
	new_pos[6] += radians;
	return goToJointPos(new_pos, std::abs(radians) / 2 / 3.14 * wrist_speed, wait);


}

bool Arm::isAtPos(const std::vector<double>& pos_vec){
	const double epsilon= 0.1;
	updateJointStatePos();
	int pos = pos_vec.size() - 7;
	for (int i=0; i<7; i++){
		printf ("curent pos %f pos_vec %f \n", current_joint_angles_[i], pos_vec[pos + i] );
		if (std::abs(current_joint_angles_[i] - pos_vec[pos + i]) > epsilon)
			return false;
	}
	return true;
}

bool Arm::tuck(){

	ROS_INFO("tucking arm %s", armside_str.c_str());
	std::vector<double> tuck_pos_vec;
	if (armside_str == "r"){
		double tuck_pos[] = { -0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.01,1.35,-1.92,-1.68, 1.35,-0.18,0.31};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+14);
	}else{
		double tuck_pos[] = {   0.4,0.0,0.0,-2.25,0.0,0.0,0.0, -0.05,1.31,1.38,-2.06,1.69,-2.02,2.44};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+14);
	}
	if (!isAtPos(tuck_pos_vec)){
		return goToJointPos(tuck_pos_vec);
	}
	ROS_INFO(" arm %s is already in tucked pos", armside_str.c_str());
	return true;

}

bool Arm::stretch(){

	ROS_INFO("stretching arm %s", armside_str.c_str());
	std::vector<double> tuck_pos_vec;
	if (armside_str == "r"){
		double tuck_pos[] = {-1.634, -0.039, -0.324, -0.131, 31.779, 0.004, 24.986};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+7);
	}else{
		double tuck_pos[] = {   1.613, -0.105, 0.336, -0.033, -6.747, 0.014, 0.295};
		tuck_pos_vec.insert(tuck_pos_vec.begin(),tuck_pos, tuck_pos+7);
	}
	if (!isAtPos(tuck_pos_vec)){
		return goToJointPos(tuck_pos_vec);
	}
	ROS_INFO(" arm %s is already in tucked pos", armside_str.c_str());
	return true;

}


tf::StampedTransform Arm::gripperToWrist(const tf::StampedTransform& pose){
	tf::Vector3 offset(gripperlength,0,0);
	tf::Transform rot(pose.getRotation());
	tf::StampedTransform out( tf::Transform(pose.getRotation(), pose.getOrigin() - rot * offset) , pose.stamp_, pose.frame_id_, pose.child_frame_id_);
	return out;
}

tf::StampedTransform Arm::wristToGripper(const tf::StampedTransform& pose){
	tf::Vector3 offset(gripperlength,0,0);
	tf::Transform rot(pose.getRotation());
	tf::StampedTransform out( tf::Transform(pose.getRotation(), pose.getOrigin() + rot * offset) , pose.stamp_, pose.frame_id_, pose.child_frame_id_);
	return out;
}


tf::StampedTransform Arm::makePose(const tf::Vector3& position, string frame_id, approach_direction_t approach){


	tf::StampedTransform tf(tf::Transform(tf::Quaternion(0,0,0,1), position), ros::Time(), frame_id, "/doesntmatter");
	tf::TransformListener tf_listener;
	tf::StampedTransform tf_sourceframe_in_baselink;

	tf_listener.waitForTransform(  "/base_link", tf.frame_id_, ros::Time(0), ros::Duration(0.5));
	tf_listener.lookupTransform(  "/base_link", tf.frame_id_, ros::Time(), tf_sourceframe_in_baselink);
	tf::StampedTransform tf_pose_in_baselink (tf::Transform(tf_sourceframe_in_baselink * tf), tf.stamp_, "/base_link", "/doesntmatter") ;

	switch (approach){

		case (FRONTAL):
										tf_pose_in_baselink.setRotation(tf::Quaternion( 0, 0 , 0, 1)); break;
		case (FROM_BELOW):
										tf_pose_in_baselink.setRotation(tf::Quaternion( HALF_SQRT_TWO , 0, HALF_SQRT_TWO , 0)); break;
		case (FROM_RIGHT_SIDEWAYS):
										tf_pose_in_baselink.setRotation(tf::Quaternion( 0 , 0, HALF_SQRT_TWO , HALF_SQRT_TWO)); break;
		case (FROM_RIGHT_UPRIGHT):
										tf_pose_in_baselink.setRotation(tf::Quaternion( -0.5 , -0.5, 0.5 , 0.5)); break;
		case (FROM_ABOVE):
										tf_pose_in_baselink.setRotation(tf::Quaternion( HALF_SQRT_TWO , 0, -HALF_SQRT_TWO , 0)); break;
		case (FROM_LEFT_SIDEWAYS):
										tf_pose_in_baselink.setRotation(tf::Quaternion( -HALF_SQRT_TWO , HALF_SQRT_TWO , 0 , 0)); break;
		case (FROM_LEFT_UPRIGHT):
										tf_pose_in_baselink.setRotation(tf::Quaternion( -0.5 , 0.5, -0.5 , 0.5)); break;
	}

	return tf_pose_in_baselink;

}


bool Arm::moveGripperToPosition(const tf::Vector3& position, string frame_id, approach_direction_t approach,  double max_time , bool wait, vector<double>* ik_seed_pos  ){

	tf::StampedTransform tf_pose_in_baselink_new(gripperToWrist(makePose(position,  frame_id,  approach)));
	return moveWristRollLinktoPose(tf_pose_in_baselink_new, max_time, wait, ik_seed_pos);
}




bool Arm::moveGrippertoPose(const tf::StampedTransform& tf, double max_time, bool wait, vector<double>* ik_seed_pos  ){


	tf::StampedTransform tf_new(gripperToWrist(tf));
	return moveWristRollLinktoPose(tf_new, max_time, wait, ik_seed_pos);

}

bool Arm::moveGrippertoPoseWithCollisionChecking(const tf::StampedTransform& tf, double max_time, bool wait, std::string planner ){


	tf::StampedTransform tf_new(gripperToWrist(tf));
	return moveWristRollLinktoPoseWithCollisionChecking(tf_new, max_time, wait, planner);

}

bool Arm::moveGrippertoPositionWithCollisionChecking(const tf::Vector3& position, string frame_id, approach_direction_t approach,  double max_time, bool wait,std::string planner ){


	tf::StampedTransform tf_pose_in_baselink_new(gripperToWrist(makePose(position,  frame_id,  approach)));
	return moveWristRollLinktoPoseWithCollisionChecking(tf_pose_in_baselink_new, max_time, wait,planner);

}

bool Arm::moveGrippertoPoseWithOrientationConstraints(const tf::StampedTransform& tf, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time, bool wait, double tolerance ){

	tf::StampedTransform tf_new(gripperToWrist(tf));
	return moveWristRollLinktoPoseWithOrientationConstraints(tf_new, keep_roll, keep_pitch, keep_yaw, max_time, wait);

}


bool Arm::moveWristRollLinktoPoseWithOrientationConstraints(const tf::StampedTransform& tf, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time, bool wait, double tolerance ){
	geometry_msgs::PoseStamped pose;
	tf::Stamped<tf::Pose> tf_pose(tf,tf.stamp_, tf.frame_id_);

	tf::poseStampedTFToMsg(tf_pose,pose);
	return moveWristRollLinktoPoseWithOrientationConstraints(pose, keep_roll, keep_pitch, keep_yaw, max_time, wait, tolerance);
}


bool Arm::moveWristRollLinktoPoseWithOrientationConstraints(const geometry_msgs::PoseStamped& pose, bool keep_roll, bool keep_pitch, bool keep_yaw, double max_time, bool wait, double tolerance ){
	if (!move_arm_client_){
		ROS_ERROR("collision checking arm server has not been started");
		return false;
	}
	bool finished_before_timeout = true;
	arm_navigation_msgs::MoveArmGoal goalA;

	goalA.motion_plan_request.group_name = group_name;
	goalA.motion_plan_request.num_planning_attempts = 1;
	goalA.motion_plan_request.planner_id = std::string("");
	goalA.planner_service_name = std::string("ompl_planning/plan_kinematic_path");
	goalA.motion_plan_request.allowed_planning_time = ros::Duration(25.0);

	goalA.motion_plan_request.goal_constraints.position_constraints.resize(1);
	goalA.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = pose.header.stamp;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id =pose.header.frame_id ;

	goalA.motion_plan_request.goal_constraints.position_constraints[0].link_name = armside_str + "_wrist_roll_link";
	goalA.motion_plan_request.goal_constraints.position_constraints[0].position.x = pose.pose.position.x;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].position.y = pose.pose.position.y;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].position.z = pose.pose.position.z;

	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);
	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back(0.02);

	goalA.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = 1.0;
	goalA.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;


	goalA.motion_plan_request.goal_constraints.position_constraints.resize(1);
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = pose.header.stamp;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = pose.header.frame_id;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = armside_str + "_wrist_roll_link";
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = pose.pose.orientation.x;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = pose.pose.orientation.y;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = pose.pose.orientation.z;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = pose.pose.orientation.w;

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;

	goalA.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;


	goalA.motion_plan_request.path_constraints.orientation_constraints.resize(1);
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].header.frame_id = pose.header.frame_id ;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].header.stamp = pose.header.stamp;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].link_name = armside_str + "_wrist_roll_link";

	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.x = pose.pose.orientation.x;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.y = pose.pose.orientation.y;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.z = pose.pose.orientation.z;
	goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.w = pose.pose.orientation.w;



	goalA.motion_plan_request.path_constraints.orientation_constraints[0].type = arm_navigation_msgs::OrientationConstraint::HEADER_FRAME;
	if (keep_roll)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_roll_tolerance = tolerance;
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_roll_tolerance = M_PI;

	if (keep_pitch)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = tolerance;
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_pitch_tolerance = M_PI;

	if (keep_yaw)
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = tolerance;
	else
		goalA.motion_plan_request.path_constraints.orientation_constraints[0].absolute_yaw_tolerance = M_PI;



	 move_arm_client_->sendGoal(goalA);

	if (wait){
		finished_before_timeout = move_arm_client_->waitForResult(ros::Duration(60.0));
		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = move_arm_client_->getState();
			ROS_INFO("Action finished: %s",state.toString().c_str());
		}
		else{
			ROS_INFO("Action did not finish before the time out.");
			return false;
		}
	}


//     if( move_arm_client_->getState() == actionlib::SimpleClientGoalState::ABORTED){
//		ROS_INFO(" trying to reach pose with inverted gripper pose");
//		tf::Transform tf_pose;
//		tf::poseMsgToTF(pose.pose, tf_pose);
//		btQuaternion rot= tf_pose.getRotation();
//		btQuaternion flip(btScalar(!keep_roll), btScalar(!keep_pitch), btScalar(!keep_yaw), 0);
//		flip.normalize();
//		rot = rot * flip;
//
//		goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = rot.x();
//		goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = rot.y();
//		goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = rot.z();
//		goalA.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = rot.w();
//
//		goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.x = rot.x();
//		goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.y = rot.z();
//		goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.z = rot.y();
//		goalA.motion_plan_request.path_constraints.orientation_constraints[0].orientation.w = rot.w();
//		move_arm_client_->sendGoal(goalA);
//		  if (wait){
//				finished_before_timeout = move_arm_client_->waitForResult(ros::Duration(60.0));
//				if (finished_before_timeout)
//				{
//					actionlib::SimpleClientGoalState state = move_arm_client_->getState();
//					ROS_INFO("Action finished: %s",state.toString().c_str());
//				}
//				else{
//					ROS_INFO("Action did not finish before the time out.");
//					return false;
//				}
//			}
//
//     }
//
////		return finished_before_timeout;
     if( move_arm_client_->getState() == actionlib::SimpleClientGoalState::ABORTED){
    	 return false;
     }else{
    	 return true;
     }

}


} //end namespace simple_robot_control
