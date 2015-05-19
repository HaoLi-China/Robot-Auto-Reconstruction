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

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

const string point_cloud_topic_="/camera/depth_registered/points";

int main(int argc, char** argv){

    if(argc != 2){
        cout<<"Number of parameter should be 1."<<endl;
        return -1;
    }

    ros::init(argc, argv, "work_before_calibration");
    ros::NodeHandle nh;

    //Create robot controller interface
    simple_robot_control::Robot robot;
    //PR2MoveArmJoint move_arm_joint;

    double pos_left_arm[] = {1.613, 0.063, 1.532, -1.536, -0.141, -0.094, -7.788};
    double pos_right_arm0[] = {0.564, 0.537, 0.133, -2.047, -10.001, -1.754, 2.978};
    double pos_right_arm1[] = {-1.609, 0.046, -1.557, -1.538, -0.009, -0.102, -4.850};
    tf::Vector3 head_focus(1.2, 0.0, 0.8);
    float torso_up = 0.20;

    if(strcmp(argv[1], "0") == 0){
        init_robot_pose(robot, pos_left_arm, pos_right_arm0, head_focus, torso_up);
    }
    else if(strcmp(argv[1], "1") == 0){
        init_robot_pose(robot, pos_left_arm, pos_right_arm1, head_focus, torso_up);
    }
    else if(strcmp(argv[1], "2") == 0){
        init_robot_pose(robot, pos_left_arm, pos_right_arm0, head_focus, torso_up);
    }

    ros::Duration(3.0).sleep();

    //load point cloud
    PointCloudPtr cloud (new PointCloud);
    PointCloudPtr cloud_filter (new PointCloud);

    // get the point cloud in
    sensor_msgs::PointCloud2ConstPtr cloud_in;
    cloud_in = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic_, nh);
    cerr<< "Received cloud: cloud time " << cloud_in->header.stamp<<endl;

    pcl::fromROSMsg(*cloud_in, *cloud);

    for(int i=0; i<cloud->size(); i++){
        if(!(isnan(cloud->points[i].x)||isnan(cloud->points[i].y)||isnan(cloud->points[i].z))){
            cloud_filter->push_back(cloud->points[i]);
        }
    }

    if(strcmp(argv[1], "0") == 0){
        //pcl::io::savePCDFile("/home/hao/groovy_ws/myws/robot_auto_reconstruction/data/handheld_kinect.pcd", *cloud);
        pcl::io::savePLYFileASCII("/home/hao/groovy_ws/myws/robot_auto_reconstruction/data/handheld_kinect.ply", *cloud_filter);
    }
    else if(strcmp(argv[1], "1") == 0){
        pcl::io::savePLYFileASCII("/home/hao/groovy_ws/myws/robot_auto_reconstruction/data/head_kinect.ply", *cloud_filter);
    }

    return 0;
}
