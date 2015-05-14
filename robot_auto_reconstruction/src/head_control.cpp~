/*
 * head_control.cpp
 *
 *  Created on: Jul 27, 2010
 *      Author: christian
 */

#include <tf/transform_datatypes.h>
#include <simple_robot_control/head_control.h>

namespace simple_robot_control{

Head::Head(){
	  traj_client_ = new actionlib::SimpleActionClient< pr2_controllers_msgs::PointHeadAction> ("head_traj_controller/point_head_action", true);

	  if (!traj_client_->waitForServer(ros::Duration(5.0))){
		  ROS_ERROR("Head: Could not find the joint_trajectory_action server");
	   }
}
Head::~Head(){
	delete traj_client_;
}




bool Head::lookat(  const string source_frame, tf::Vector3 position , double speed, string pointing_frame ){


	pr2_controllers_msgs::PointHeadGoal goal;
	goal.pointing_frame = pointing_frame;
	goal.pointing_axis.x = 1;
	goal.pointing_axis.y = 0;
	goal.pointing_axis.z = 0;

	goal.target.header.frame_id = source_frame;
	goal.target.point.x = position.x();
	goal.target.point.y = position.y();
	goal.target.point.z = position.z();

    goal.min_duration = ros::Duration(0.5);
    goal.max_velocity = speed;

	traj_client_->sendGoal(goal);
	bool success = traj_client_->waitForResult(ros::Duration(10.0));
	if (!success){
		ROS_ERROR("Could not point head");
		return false;
	}
	ROS_INFO("moved head");
	return true;

}



bool Head::lookat(  const  geometry_msgs::PoseStampedConstPtr pose, double speed, string pointing_frame){

	return lookat(pose->header.frame_id, tf::Vector3(pose->pose.position.x, pose->pose.position.y, pose->pose.position.z), speed, pointing_frame);
}




bool Head::lookat( const string source_frame, double x, double y, double z, double speed, string pointing_frame ){

	return lookat(source_frame, tf::Vector3(x,y,z), speed, pointing_frame);
}


bool Head::lookat( const tf::StampedTransform  pose, double speed, string pointing_frame){

	return lookat(pose.frame_id_, pose.getOrigin(), speed, pointing_frame);
}


}
