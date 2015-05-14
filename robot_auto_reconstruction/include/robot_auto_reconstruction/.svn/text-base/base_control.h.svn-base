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
 * simple_base_control.h
 *
 *  Created on: Sep 20, 2010
 *      Author: has2pal
 */

#ifndef SIMPLE_BASE_CONTROL_H_
#define SIMPLE_BASE_CONTROL_H_

#include <cmath>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>


namespace simple_robot_control{

class Base {
private:
	ros::NodeHandle nh_;

	geometry_msgs::Twist cmd_;
	ros::Publisher pub_base_vel_;
	tf::TransformListener tf_listener_odom_;

	static const double PUBLISH_RATE_ = 20.0;

	void init();

public:
	Base();
	virtual ~Base();

	// Drive forward a specified distance in m with given speed in m/s
	bool driveForward(double distance, double speed = 0.25);

	// Drive right a specified distance in m with given speed in m/s
	bool driveRight(double distance, double speed = 0.25);

	// Drive back a specified distance in m with given speed in m/s
	bool driveBack(double distance, double speed = 0.25);

	// Drive left a specified distance in m with given speed in m/s
	bool driveLeft(double distance, double speed = 0.25);

	// Drive a specified distance (based on base odometry information) with given velocity Twist (vector)
	bool drive(double distance, const geometry_msgs::Twist& velocity);

	// Rotate as specified in rad (based on base odometry information) with given speed in rad/s
	bool turn(bool clockwise, double radians, double speed = 0.75);
};

}
#endif /* SIMPLE_BASE_CONTROL_H_ */
