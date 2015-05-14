/*
 * simple_base_control.cpp
 *
 *  Created on: Sep 10, 2010
 *      Author: Sebastian Haug | Bosch RTC
 *
 *  Controls PR2 Base. Provides easy to use high level interface
 *  to move and rotate base. No further services have to be launched.
 *
 *  How to use:
 *     - Configured as Library. Add dependency to in simple_base_control_cpp
 *       in own ROS package and instantiate SimpleBaseControl object.
 */

#include "simple_robot_control/base_control.h"

namespace simple_robot_control{

Base::Base()
{
	init();
}

Base::~Base()
{ }

void Base::init()
{
	// init cmd_
	cmd_.linear.x = cmd_.linear.y = cmd_.angular.z = 0;

	// register publisher
	pub_base_vel_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);
}

// Drive forward a specified distance in m with given speed in m/s
bool Base::driveForward(double distance, double speed)
{
	geometry_msgs::Twist vel;
	vel.linear.x = std::abs(speed);

	return drive(distance, vel);
}

// Drive right a specified distance in m with given speed in m/s
bool Base::driveRight(double distance, double speed)
{
	geometry_msgs::Twist vel;
	vel.linear.y = -std::abs(speed);

	return drive(distance, vel);
}

// Drive back a specified distance in m with given speed in m/s
bool Base::driveBack(double distance, double speed)
{
	geometry_msgs::Twist vel;
	vel.linear.x = -std::abs(speed);

	return drive(distance, vel);
}

// Drive left a specified distance in m with given speed in m/s
bool Base::driveLeft(double distance, double speed)
{
	geometry_msgs::Twist vel;
	vel.linear.y = std::abs(speed);

	return drive(distance, vel);
}

// Drive a specified distance (based on base odometry information) with given velocity Twist (vector)
bool Base::drive(double distance, const geometry_msgs::Twist& velocity)
{
	tf::StampedTransform start_transform;
	tf::StampedTransform current_transform;

	// Wait and get transform
	tf_listener_odom_.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(5.0));
	tf_listener_odom_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), start_transform);

	// Set cmd_ to velocity and clear angular and linear.z;
	cmd_ = velocity;
	cmd_.angular.x = cmd_.angular.y = cmd_.angular.z = cmd_.linear.z = 0;



	// Loop until pos reached
	ros::Rate rate(PUBLISH_RATE_);
	bool done = false;

	while (!done && nh_.ok()) {
		// Send the drive command
		pub_base_vel_.publish(cmd_);
		rate.sleep();

		// Get the current transform
		try {
			tf_listener_odom_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), current_transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			break;
		}

		// See how far we've traveled
		tf::Transform relative_transform = start_transform.inverse() * current_transform;
		double dist_moved = relative_transform.getOrigin().length();


		if (dist_moved >= distance)
			done = true;
	}
	return done;
}

// Rotate as specified in rad (based on base odometry information) with given speed in rad/s
bool Base::turn(bool clockwise, double radians, double speed)
{
	while (radians < 0)			radians += 2 * M_PI;
	while (radians > 2 * M_PI)	radians -= 2 * M_PI;

	tf::StampedTransform start_transform;
	tf::StampedTransform current_transform;

	// Wait and get transform
	tf_listener_odom_.waitForTransform("base_footprint", "odom_combined", ros::Time(0), ros::Duration(5.0));
	tf_listener_odom_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), start_transform);

	// Set cmd_
	cmd_.linear.x = cmd_.linear.y = cmd_.linear.z = 0.0;
	cmd_.angular.z = speed;
	if (clockwise)
		cmd_.angular.z = -speed ;



	// The axis we want to be rotating by
	tf::Vector3 desired_turn_axis(0, 0, 1);
	if (!clockwise)
		desired_turn_axis = -desired_turn_axis;

	// Loop until finished turn
	ros::Rate rate(PUBLISH_RATE_);
	bool done = false;

	while (!done && nh_.ok()) {
		// Send the drive command
		pub_base_vel_.publish(cmd_);
		rate.sleep();

		// Get the current transform
		try {
			tf_listener_odom_.lookupTransform("base_footprint", "odom_combined", ros::Time(0), current_transform);
		} catch (tf::TransformException ex) {
			ROS_ERROR("%s",ex.what());
			break;
		}
		tf::Transform relative_transform = start_transform.inverse() * current_transform;
		tf::Vector3 actual_turn_axis = relative_transform.getRotation().getAxis();
		double angle_turned = relative_transform.getRotation().getAngle();
		if (fabs(angle_turned) < 1.0e-2)
			continue;

		if (actual_turn_axis.dot(desired_turn_axis) < 0)
			angle_turned = 2 * M_PI - angle_turned;

		if (angle_turned > radians)
			done = true;
	}
	return done;
}

}
