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

void getHandheldKinect2RGripperTF(ros::NodeHandle &nh, tf::Vector3 &translation0, tf::Quaternion &quaternion0, tf::StampedTransform &stransform){

    tf::TransformBroadcaster br0;
    tf::Transform transform0;

    tf::TransformListener listener0;

    std::stringstream stt0;
    stt0 << "handheld_kinect_frame";

    std::string frame1="head_mount_kinect_ir_optical_frame";
    std::string frame2=stt0.str();
    std::string frame3="r_gripper_tool_frame";

    int flag=0;

    while(!flag){
        ros::Time time=ros::Time::now();
        if (nh.ok()){
            transform0.setOrigin(translation0);
            transform0.setRotation(quaternion0);
            br0.sendTransform(tf::StampedTransform(transform0, time, frame1, frame2));
        }
        else{
            std::cerr<<"node is not ok!"<<std::endl;
            return;
        }

        try{
            listener0.lookupTransform(frame3,frame2,
                                     ros::Time(0), stransform);
            flag=1;
        }
        catch (tf::TransformException ex){
            // cerr<<ex.what()<<endl;
        }
    }
}

int main(int argc, char** argv){

    ros::init(argc, argv, "calibration_node");
    ros::NodeHandle nh;

//    tf::Quaternion quaternion;
//    quaternion.setX(-0.0350438);
//    quaternion.setY(-0.0400501);
//    quaternion.setZ(-0.0100125);
//    quaternion.setW(0.998749);

//    tf::Vector3 translation(0.04, 0.22, 0.75);

    tf::Quaternion quaternion;
    quaternion.setX(-0.027569);
    quaternion.setY(-0.0701757);
    quaternion.setZ(-0.00501255);
    quaternion.setW(0.997497);

    tf::Vector3 translation(0.0, 0.22, 0.75);

    tf::StampedTransform stransform;
    getHandheldKinect2RGripperTF(nh, translation, quaternion, stransform);

    cout<<"stransform.getRotation().x():"<<stransform.getRotation().x()<<endl;
    cout<<"stransform.getRotation().y():"<<stransform.getRotation().y()<<endl;
    cout<<"stransform.getRotation().z():"<<stransform.getRotation().z()<<endl;
    cout<<"stransform.getRotation().w():"<<stransform.getRotation().w()<<endl;
    cout<<"stransform.getOrigin().x():"<<stransform.getOrigin().x()<<endl;
    cout<<"stransform.getOrigin().y():"<<stransform.getOrigin().y()<<endl;
    cout<<"stransform.getOrigin().z():"<<stransform.getOrigin().z()<<endl;

    tf::Quaternion quaternion0;
    quaternion0.setX(stransform.getRotation().x());
    quaternion0.setY(stransform.getRotation().y());
    quaternion0.setZ(stransform.getRotation().z());
    quaternion0.setW(stransform.getRotation().w());

    tf::Vector3 translation0(stransform.getOrigin().x(), stransform.getOrigin().y(), stransform.getOrigin().z());

    tf::TransformBroadcaster br0;
    tf::Transform transform0;

    std::stringstream stt0;
    stt0 << "handheld_kinect_frame";

    std::string frame1="r_gripper_tool_frame";
    std::string frame2=stt0.str();

    while(nh.ok()){
        ros::Time time=ros::Time::now();
        transform0.setOrigin(translation0);
        transform0.setRotation(quaternion0);
        br0.sendTransform(tf::StampedTransform(transform0, time, frame1, frame2));

        ros::Duration(1.0).sleep();
    }

    return 0;
}
