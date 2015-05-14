/*
 * torso_control.h
 *
 *  Created on: Jun 30, 2010
 *      Author: christian
 */

#ifndef TORSO_CONTROL_H_
#define TORSO_CONTROL_H_
#include <ros/ros.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <actionlib/client/simple_action_client.h>




namespace simple_robot_control{

class Torso
{
private:
	actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> torso_client_;

public:
	//Action client initialization
	Torso();
	~Torso();

	  //tell the torso to go to the specified position between 0.0 and 0.3.
	 bool move(double position, bool wait = true);

};

}

#endif /* TORSO_CONTROL_H_ */
