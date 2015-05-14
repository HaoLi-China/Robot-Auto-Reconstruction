/*
 * robot_control.h
 *
 *  Created on: Jan 10, 2011
 *      Author: christian
 */

#ifndef ROBOT_CONTROL_H_
#define ROBOT_CONTROL_H_

#include <robot_auto_reconstruction/arm_control.h>
#include <robot_auto_reconstruction/gripper_control.h>
#include <robot_auto_reconstruction/torso_control.h>
#include <robot_auto_reconstruction/base_control.h>
#include <robot_auto_reconstruction/head_control.h>

namespace simple_robot_control{

class Robot{
public:
	Torso torso;
	Head head;
	Base base;
	Gripper left_gripper;
	Gripper right_gripper;
	Arm left_arm;
	Arm right_arm;

	Robot();

};


}


#endif /* ROBOT_CONTROL_H_ */
