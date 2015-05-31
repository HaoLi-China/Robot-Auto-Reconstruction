//just for test
#include <ros/ros.h>
#include <robot_auto_reconstruction/robot_control.h>
#include <math.h>

#include <pcl/ros/conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>

#include <sstream>

#include "auto_reconstruction.h"

using namespace std;

int main(int argc, char** argv){

    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

    //    position_kinect.x()0.170195
    //    position_kinect.y()0.014465
    //    position_kinect.z()0.506418
    //    dir_kinect.x()0.798224
    //    dir_kinect.y()-0.317258
    //    dir_kinect.z()0.512041

    //    position_kinect.x()0.035621
    //    position_kinect.y()-0.039956
    //    position_kinect.z()0.663051
    //    dir_kinect.x()0.884022
    //    dir_kinect.y()-0.283642
    //    dir_kinect.z()0.371553

    //    position_kinect.x()-0.048688
    //    position_kinect.y()-0.021600
    //    position_kinect.z()0.697898
    //    dir_kinect.x()0.530493
    //    dir_kinect.y()-0.394864
    //    dir_kinect.z()0.750106



    simple_robot_control::Robot robot;

//    double pos_left_arm[] = {1.613, 0.063, 1.532, -1.536, -0.141, -0.094, -7.788};
//    double pos_right_arm[] = {0.510, 0.625, 0.100, -2.117, 3.207, -1.854, -3.276};
//    tf::Vector3 head_focus(1.5, 0.5, 0.6);
//    double torso_up = 0.20;

//    //init_robot_pose(robot, pos_left_arm, pos_right_arm, head_focus, torso_up);

//    tf::Vector3 position_kinect(-0.048688, -0.021600, 0.697898);
//    tf::Vector3 dir_kinect(0.530493, -0.394864, 0.750106);

//    test_calibration_result(nh, position_kinect, dir_kinect);

    base_drive_forward(robot, 0.31);

    return 0;
}
