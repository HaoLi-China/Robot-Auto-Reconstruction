#include "auto_reconstruction.h"

// create quaternion with angle and axis
void CQuaternion(tf::Vector3& axis, const float& angle, tf::Quaternion& quaternion)
{
    tf::Vector3 t;
    t[0] = axis.x();
    t[1] = axis.y();
    t[2] = axis.z();
    t.normalize();
    float cosa = cos(angle/2);
    float sina = sin(angle/2);
    quaternion.setW(cosa);
    quaternion.setX(sina * t.x());
    quaternion.setY(sina * t.y());
    quaternion.setZ(sina * t.z());
}

//init robot pose
void init_robot_pose(simple_robot_control::Robot &robot, double pos_left_arm[], double pos_right_arm[], tf::Vector3 &head_focus, float torso_up){
    //look straight
    robot.head.lookat("base_link", head_focus);

    //do stuff with arms
    robot.left_arm.stretch(pos_left_arm);
    robot.right_arm.stretch(pos_right_arm);

    robot.torso.move(torso_up, true);
}

//init left arm pose
void init_left_arm_pose(simple_robot_control::Robot &robot, double pos_left_arm[]){
    robot.left_arm.stretch(pos_left_arm);
}

//init right arm pose
void init_right_arm_pose(simple_robot_control::Robot &robot, double pos_right_arm[]){
    robot.right_arm.stretch(pos_right_arm);
}


//r_arm pick up kinect
void r_pick_up_kinect(simple_robot_control::Arm& arm_r, simple_robot_control::Gripper& gripper_r, simple_robot_control::Base& base){
    ros::Duration(1.0).sleep();

    ros::Duration(1.0).sleep();

    base.driveBack(0.4, 0.05);

    tf::StampedTransform tf_kinect_up_r1 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_up_r1");
    arm_r.moveGrippertoPose(tf_kinect_up_r1);
    gripper_r.open(-1.0);

    tf::StampedTransform tf_kinect_up_r2 (tf::Transform(tf::Quaternion(-0.015, 0.999, 0.033, 0.011), tf::Vector3(0.099, -0.007, 0.428)), ros::Time::now(), "base_link","tf_kinect_up_r2");
    arm_r.moveGrippertoPose(tf_kinect_up_r2);
    gripper_r.close(100);

    tf::StampedTransform tf_kinect_up_r3 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_up_r3");
    arm_r.moveGrippertoPose(tf_kinect_up_r3);

    tf::StampedTransform tf_kinect_up_r4 (tf::Transform(tf::Quaternion(-0.008, -0.020, 0.040, 0.999), tf::Vector3(0.469, -0.703, 1.027)), ros::Time::now(), "base_link","tf_kinect_up_r4");
    arm_r.moveGrippertoPose(tf_kinect_up_r4);

    ros::Duration(1.0).sleep();

    base.driveForward(0.4, 0.05);
}

//r_arm put down kinect
void r_put_down_kinect(simple_robot_control::Arm& arm_r, simple_robot_control::Gripper& gripper_r, simple_robot_control::Base& base){
    base.driveBack(0.4, 0.05);

    ros::Duration(1.0).sleep();

    tf::StampedTransform tf_kinect_down_r1 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_down_r1");
    arm_r.moveGrippertoPose(tf_kinect_down_r1);

    tf::StampedTransform tf_kinect_down_r2 (tf::Transform(tf::Quaternion(-0.015, 0.999, 0.033, 0.011), tf::Vector3(0.099, -0.007, 0.428)), ros::Time::now(), "base_link","tf_kinect_down_r2");
    arm_r.moveGrippertoPose(tf_kinect_down_r2);
    gripper_r.open(-1.0);

    tf::StampedTransform tf_kinect_down_r3 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_down_r3");
    arm_r.moveGrippertoPose(tf_kinect_down_r3);
    gripper_r.close(100);

    arm_r.stretch();

    ros::Duration(1.0).sleep();

    base.driveForward(0.4, 0.05);
}

//l_arm pick up kinect
void l_pick_up_kinect(simple_robot_control::Arm& arm_l, simple_robot_control::Gripper& gripper_l, simple_robot_control::Base& base){
    base.driveBack(0.4, 0.05);

    ros::Duration(1.0).sleep();

    tf::StampedTransform tf_kinect_up_l1 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_up_l1");
    arm_l.moveGrippertoPose(tf_kinect_up_l1);
    gripper_l.open(-1.0);

    tf::StampedTransform tf_kinect_up_l2 (tf::Transform(tf::Quaternion(-0.015, 0.999, 0.033, 0.011), tf::Vector3(0.099, -0.007, 0.428)), ros::Time::now(), "base_link","tf_kinect_up_l2");
    arm_l.moveGrippertoPose(tf_kinect_up_l2);
    gripper_l.close(100);

    tf::StampedTransform tf_kinect_up_l3 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_up_l3");
    arm_l.moveGrippertoPose(tf_kinect_up_l3);

    tf::StampedTransform tf_kinect_up_l4 (tf::Transform(tf::Quaternion(-0.106, -0.014, -0.053, 0.993), tf::Vector3(0.456, 0.667, 0.996)), ros::Time::now(), "base_link","tf_kinect_up_l4");
    arm_l.moveGrippertoPose(tf_kinect_up_l4);

    ros::Duration(1.0).sleep();

    base.driveForward(0.4, 0.05);
}

//l_arm put down kinect
void l_put_down_kinect(simple_robot_control::Arm& arm_l, simple_robot_control::Gripper& gripper_l, simple_robot_control::Base& base){
    base.driveBack(0.4, 0.05);

    ros::Duration(1.0).sleep();

    tf::StampedTransform tf_kinect_down_l1 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_down_l1");
    arm_l.moveGrippertoPose(tf_kinect_down_l1);

    tf::StampedTransform tf_kinect_down_l2 (tf::Transform(tf::Quaternion(-0.015, 0.999, 0.033, 0.011), tf::Vector3(0.099, -0.007, 0.428)), ros::Time::now(), "base_link","tf_kinect_down_l2");
    arm_l.moveGrippertoPose(tf_kinect_down_l2);
    gripper_l.open(-1.0);

    tf::StampedTransform tf_kinect_down_l3 (tf::Transform(tf::Quaternion(-0.018, 1.000, 0.022, 0.008), tf::Vector3(0.130, 0.002, 0.548)), ros::Time::now(), "base_link","tf_kinect_down_l3");
    arm_l.moveGrippertoPose(tf_kinect_down_l3);
    gripper_l.close(100);

    arm_l.stretch();

    ros::Duration(1.0).sleep();

    base.driveForward(0.4, 0.05);
}

//left arm push object
void l_push_object(simple_robot_control::Arm& arm_l, tf::Vector3& position, tf::Vector3& direction){
    tf::Vector3 axis(0, 0, 1);
    tf::Vector3 push_direction(direction.x(), direction.y(), 0);
    push_direction.normalize();

    float angle = acos(push_direction.dot(tf::Vector3(1, 0, 0)));

    tf::Quaternion quaternion;
    CQuaternion(axis, angle, quaternion);

    std::cout<<  "quaternion.w():" << quaternion.w() <<std::endl;
    std::cout<<  "quaternion.x():" << quaternion.x() <<std::endl;
    std::cout<<  "quaternion.y():" << quaternion.y() <<std::endl;
    std::cout<<  "quaternion.z():" << quaternion.z() <<std::endl;

    tf::Vector3 start_position = position - 0.03*push_direction;
    tf::Vector3 target_position = position + 0.1*push_direction;


    //use left arm
    tf::StampedTransform tf_l1 (tf::Transform(quaternion, start_position), ros::Time::now(), "base_link","tf_l1");
    arm_l.moveGrippertoPose(tf_l1);

    //arm_l.moveGripperToPosition(tf::Vector3(0.741,-0.413, 0.809), "base_link", simple_robot_control::Arm::FRONTAL);

    tf::StampedTransform tf_l2 (tf::Transform(quaternion, target_position), ros::Time::now(), "base_link","tf_l2");
    arm_l.moveGrippertoPose(tf_l2);

    ros::Duration(1.0).sleep();
}

//right arm push object
void r_push_object(simple_robot_control::Arm& arm_r, tf::Vector3& position, tf::Vector3& direction){
    tf::Vector3 axis(0, 0, 1);
    tf::Vector3 push_direction(direction.x(), direction.y(), 0);
    push_direction.normalize();

    float angle = acos(push_direction.dot(tf::Vector3(1, 0, 0)));

    tf::Quaternion quaternion;
    CQuaternion(axis, angle, quaternion);

    std::cout<<  "quaternion.w():" << quaternion.w() <<std::endl;
    std::cout<<  "quaternion.x():" << quaternion.x() <<std::endl;
    std::cout<<  "quaternion.y():" << quaternion.y() <<std::endl;
    std::cout<<  "quaternion.z():" << quaternion.z() <<std::endl;

    tf::Vector3 start_position = position - 0.03*push_direction;
    tf::Vector3 target_position = position + 0.1*push_direction;

    //use right arm
    tf::StampedTransform tf_r1 (tf::Transform(quaternion, start_position), ros::Time::now(), "base_link","tf_r1");
    arm_r.moveGrippertoPose(tf_r1);

    tf::StampedTransform tf_r2 (tf::Transform(quaternion, target_position), ros::Time::now(), "base_link","tf_r2");
    arm_r.moveGrippertoPose(tf_r2);
    ros::Duration(1.0).sleep();
}

//left arm take back
void l_take_back(simple_robot_control::Arm& arm_l, tf::Vector3& position, tf::Vector3& direction){
    tf::Vector3 axis(0, 0, 1);
    tf::Vector3 push_direction(direction.x(), direction.y(), 0);
    push_direction.normalize();

    float angle = acos(push_direction.dot(tf::Vector3(1, 0, 0)));

    tf::Quaternion quaternion;
    CQuaternion(axis, angle, quaternion);

    std::cout<<  "quaternion.w():" << quaternion.w() <<std::endl;
    std::cout<<  "quaternion.x():" << quaternion.x() <<std::endl;
    std::cout<<  "quaternion.y():" << quaternion.y() <<std::endl;
    std::cout<<  "quaternion.z():" << quaternion.z() <<std::endl;

    tf::Vector3 start_position = position + 0.1*push_direction;
    tf::Vector3 target_position = position - 0.03*push_direction;

    //use left arm
    tf::StampedTransform tf_l1 (tf::Transform(quaternion, start_position), ros::Time::now(), "base_link","tf_l1");
    arm_l.moveGrippertoPose(tf_l1);

    //arm_l.moveGripperToPosition(tf::Vector3(0.741,-0.413, 0.809), "base_link", simple_robot_control::Arm::FRONTAL);

    tf::StampedTransform tf_l2 (tf::Transform(quaternion, target_position), ros::Time::now(), "base_link","tf_l2");
    arm_l.moveGrippertoPose(tf_l2);

    ros::Duration(1.0).sleep();
}

//right arm take back
void r_take_back(simple_robot_control::Arm& arm_r, tf::Vector3& position, tf::Vector3& direction){
    tf::Vector3 axis(0, 0, 1);
    tf::Vector3 push_direction(direction.x(), direction.y(), 0);
    push_direction.normalize();

    float angle = acos(push_direction.dot(tf::Vector3(1, 0, 0)));

    tf::Quaternion quaternion;
    CQuaternion(axis, angle, quaternion);

    std::cout<<  "quaternion.w():" << quaternion.w() <<std::endl;
    std::cout<<  "quaternion.x():" << quaternion.x() <<std::endl;
    std::cout<<  "quaternion.y():" << quaternion.y() <<std::endl;
    std::cout<<  "quaternion.z():" << quaternion.z() <<std::endl;

    tf::Vector3 start_position = position + 0.1*push_direction;
    tf::Vector3 target_position = position - 0.03*push_direction;

    //use right arm
    tf::StampedTransform tf_r1 (tf::Transform(quaternion, start_position), ros::Time::now(), "base_link","tf_r1");
    arm_r.moveGrippertoPose(tf_r1);

    tf::StampedTransform tf_r2 (tf::Transform(quaternion, target_position), ros::Time::now(), "base_link","tf_r2");
    arm_r.moveGrippertoPose(tf_r2);
    ros::Duration(1.0).sleep();
}

//make left scanner up and down
void up_down_left_scanner(PR2MoveArmJoint &move_arm_joint, float down_value, float up_value){
    for (int i = 0; i < 2; ++i)
    {
        move_arm_joint.startTrajectory(move_arm_joint.armScanTrajectory(0, down_value, up_value));
        while(!move_arm_joint.getState().isDone() && ros::ok())
        {
            usleep(1000);
        }
    }
}

//make right scanner up and down
void up_down_right_scanner(PR2MoveArmJoint &move_arm_joint, float down_value, float up_value){
    for (int i = 0; i < 2; ++i)
    {
        move_arm_joint.startTrajectory(move_arm_joint.armScanTrajectory(1, down_value, up_value));
        while(!move_arm_joint.getState().isDone() && ros::ok())
        {
            usleep(1000);
        }
    }
}

//set head pose
void set_head_pose(simple_robot_control::Head& head, tf::Vector3 &head_focus){
    //look straight
    head.lookat("base_link", head_focus);
}





//int main(int argc, char** argv){
//    ros::init(argc, argv, "robot_control_test_app");
//    ros::NodeHandle nh;

//    //Create robot controller interface
//    simple_robot_control::Robot robot;

//    //look straight

//    robot.head.lookat("base_link", tf::Vector3(0.5, 0.0, 1.2));
//    //robot.head.lookat("base_link", tf::Vector3(0.668, 0.150, 0.764));

////    double pos_right[] = {-0.955,0.366,-0.312,-1.963,4.814,-1.660,3.018};
////    std::vector<double> pos_vec(pos_right, pos_right+7);
////    robot.right_arm.goToJointPos(pos_vec);

////    double pos_left[] = {1.492,1.296,1.506,-1.670,-0.306,-1.476,-1.600};
////    std::vector<double> pos_vec(pos_left, pos_left+7);
////    robot.left_arm.goToJointPos(pos_vec);


//    //do stuff with arms
//    //robot.left_arm.stretch();
//    //robot.right_arm.stretch();

//    //ros::Duration(6.0).sleep();

//    robot.torso.move(0.28, false);

//    ros::Duration(1.0).sleep();

////    //r_arm pick up kinect
////    r_pick_up_kinect(robot.right_arm, robot.right_gripper, robot.base);
////    //r_arm put down kinect
////    r_put_down_kinect(robot.right_arm, robot.right_gripper, robot.base);
////    //l_arm pick up kinect
////    l_pick_up_kinect(robot.left_arm, robot.left_gripper, robot.base);
////    //l_arm put down kinect
////    l_put_down_kinect(robot.left_arm, robot.left_gripper, robot.base);



//    //robot.base.driveLeft(1.2, 0.04);

//    //robot.base.driveRight(1.2, 0.04);
//    //robot.base.driveForward(0.15, 0.04);

//    //robot.base.driveBack(0.15, 0.04);

//    //robot.base.turn(1, M_PI, 0.1);

//    //robot.base.driveRight(3.5, 0.06);
//    //robot.base.driveBack(0.5, 0.06);
//    //robot.base.turn(1, M_PI, 0.1);


//    //robot.base.circle_turn (0, 0.5, 0.06);





////    tf::Vector3 positon1(0.638, 0.552, 0.772);
////    tf::Vector3 direction1(0.6,0.4,0);
////    push_object(robot.left_arm, robot.right_arm, positon1, direction1);


////    tf::StampedTransform tf_kinect (tf::Transform(tf::Quaternion(-0.083, 0.176, 0.347, 0.918), tf::Vector3(0.465, 0.172, 1.054)), ros::Time::now(), "base_link","tf_kinect");
////    robot.right_arm.moveGrippertoPose(tf_kinect);



//    //    tf::Vector3 positon2(0.315792, 0.712564, 0.760689);
//    //    tf::Vector3 direction2(0.74809, 0.663041, -0.0271604);
//    //    push_object(robot.left_arm, robot.right_arm, positon2, direction2);

////    tf::Vector3 positon2(0.375792, 0.562564, 0.760689);
////    tf::Vector3 direction2(0.74809, 0.663041, -0.0271604);
////    push_object(robot.left_arm, robot.right_arm, positon2, direction2);


//    //robot.base.driveForward(0.05, 0.05);
//    //robot.base.driveLeft(0.8, 0.05);
//    //robot.base.driveBack(0.1, 0.05);
//    //robot.base.driveRight(0.1, 0.05);

//    //    double tuck_pos_right[] = { -0.4,1.0,0.0,-2.05,0.0,-0.1,0.0, -0.024,1.10,-1.56,-2.12, -1.42, -1.84, 0.21};
//    //    std::vector<double> tuck_pos_vec(tuck_pos_right, tuck_pos_right+14);
//    //    robot.right_arm.goToJointPos(tuck_pos_vec);

//    //    //robot.right_arm.stretch();
//    //    ros::Duration(2.0).sleep();

//   //robot.left_arm.moveGripperToPosition(tf::Vector3(0.741,-0.413, 0.809), "base_link", simple_robot_control::Arm::FRONTAL);
//    //    robot.right_arm.moveGripperToPosition(tput downf::Vector3(0.787,0.081, 0.771), "base_link", simple_robot_control::Arm::FRONTAL);





//    /*********move left_arm and right_arm*********/
//    //    tf::StampedTransform tf_l (tf::Transform(tf::Quaternion(0.996, -0.036, -0.081, -0.027), tf::Vector3(0.787, 0.081, 0.771)), ros::Time::now(), "base_link","tf_l");
//    //    robot.left_arm.moveGrippertoPose(tf_l);

//    //    tf::StampedTransform tf_r (tf::Transform(tf::Quaternion(0.983, 0.155, -0.096, -0.010), tf::Vector3(0.741, -0.413, 0.809)), ros::Time::now(), "base_link","tf_r");
//    //    robot.right_arm.moveGrippertoPose(tf_r);

//    //    ros::Duration(1.0).sleep();
//    //    //look at right gripper
//    //    robot.head.lookat("r_gripper_tool_frame");

//    //drive 0.5m forward
//    //robot.base.driveForward(0.3);

//    //raise torso to 10cm above lowest position
//    //robot.torso.move(0.1);

//    return 0;

//}
