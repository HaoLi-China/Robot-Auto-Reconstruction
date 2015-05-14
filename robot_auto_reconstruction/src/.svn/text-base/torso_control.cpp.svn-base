/*
 * torso_control.cpp
 *
 *  Created on: Jul 27, 2010
 *      Author: christian
 */


#include <simple_robot_control/torso_control.h>


namespace simple_robot_control{

Torso::Torso():torso_client_ ("torso_controller/position_joint_action", true){



	//wait for the action server to come up
	while(!torso_client_.waitForServer(ros::Duration(5.0))){
		ROS_INFO("Waiting for the torso action server to come up");
	}
}

Torso::~Torso(){

}



bool Torso::move(double position, bool wait)
{
	position = (position > 0.3) ? 0.3 : position;
	position = (position < 0.0) ? 0.0 : position;

	pr2_controllers_msgs::SingleJointPositionGoal move;
	move.position = position;  //all the way up is 0.3
	move.min_duration = ros::Duration(2.0);
	move.max_velocity = 1.0;
	torso_client_.sendGoal(move);

	if (wait){
		return torso_client_.waitForResult(ros::Duration(20.0));
	}

	return true;
}

}
