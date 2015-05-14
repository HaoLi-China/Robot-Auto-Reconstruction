/*
 * head_control.h
 *
 *  Created on: Jul 6, 2010
 *      Author: christian
 */

#ifndef HEAD_CONTROL_H_
#define HEAD_CONTROL_H_


#include <string>

#include <ros/ros.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>


namespace simple_robot_control{

using std::string;

static const string  DEFAULT_HEAD_POINTING_FRAME = "high_def_frame";

class Head{
private:
	actionlib::SimpleActionClient< pr2_controllers_msgs::PointHeadAction>*  traj_client_;

	const static double DEFAULT_SPEED = 1.0;

public:
	Head();
	~Head();

	//Pose sets head position to position specified relative to the header frame id
	bool lookat( const  geometry_msgs::PoseStampedConstPtr pose, double speed = DEFAULT_SPEED, string pointing_frame = DEFAULT_HEAD_POINTING_FRAME);
	bool lookat( const tf::StampedTransform  pose, double speed = DEFAULT_SPEED, string pointing_frame = DEFAULT_HEAD_POINTING_FRAME);
	bool lookat( const string source_frame, double x, double y, double z, double speed = DEFAULT_SPEED, string pointing_frame = DEFAULT_HEAD_POINTING_FRAME);
	bool lookat( const string source_frame, tf::Vector3 position = tf::Vector3() , double speed = DEFAULT_SPEED, string pointing_frame = DEFAULT_HEAD_POINTING_FRAME);
};

}

#endif /* HEAD_CONTROL_H_ */
