#include "auto_reconstruction.h"

// create quaternion with angle and axis
void CQuaternion(const tf::Vector3& axis, const float& angle, tf::Quaternion& quaternion)
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

// create quaternion with Euler angles
void Eulerangles2Quaternion(const tf::Vector3& angle, tf::Quaternion& quaternion)
{
    float cx = cos(angle.x()/2);
    float sx = sin(angle.x()/2);
    float cy = cos(angle.y()/2);
    float sy = sin(angle.y()/2);
    float cz = cos(angle.z()/2);
    float sz = sin(angle.z()/2);

    quaternion.setW(cx*cy*cz + sx*sy*sz);
    quaternion.setX(sx*cy*cz - cx*sy*sz);
    quaternion.setY(cx*sy*cz + sx*cy*sz);
    quaternion.setZ(cx*cy*sz - sx*sy*cz);
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

    position.setZ(position.z()-0.15);
    //position.setY(position.y()-0.01);

    tf::Vector3 start_position = position - 0.35*push_direction;
    tf::Vector3 target_position = position - 0.25*push_direction;

    std::cout<<  "start_position.x():" << start_position.x() <<std::endl;
    std::cout<<  "start_position.y():" << start_position.y() <<std::endl;
    std::cout<<  "start_position.z():" << start_position.z() <<std::endl;
    std::cout<<  "target_position.x():" << target_position.x() <<std::endl;
    std::cout<<  "target_position.y():" << target_position.y() <<std::endl;
    std::cout<<  "target_position.z():" << target_position.z() <<std::endl;



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

    //tf::Vector3 start_position = position - 0.03*push_direction;
    tf::Vector3 start_position = position - 0.2*push_direction;
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

    position.setZ(position.z()-0.15);
    //position.setY(position.y()-0.01);

    tf::Vector3 start_position = position - 0.25*push_direction;
    //tf::Vector3 target_position = position - 0.03*push_direction;
    tf::Vector3 target_position = position - 0.35*push_direction;

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
    //tf::Vector3 target_position = position - 0.03*push_direction;
    tf::Vector3 target_position = position - 0.20*push_direction;

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

//just for test
void test_calibration_result(ros::NodeHandle &n, const tf::Vector3 &position_kinect, const tf::Vector3 &dir_kinect){
    //    stransform.getRotation().x():0.601777
    //    stransform.getRotation().y():-0.503993
    //    stransform.getRotation().z():0.370892
    //    stransform.getRotation().w():-0.49628
    //    stransform.getOrigin().x():0.00206023
    //    stransform.getOrigin().y():0.00838143
    //    stransform.getOrigin().z():0.113836

    //    stransform.getRotation().x():-0.606064
    //    stransform.getRotation().y():0.487114
    //    stransform.getRotation().z():-0.355795
    //    stransform.getRotation().w():0.518476
    //    stransform.getOrigin().x():-0.00336185
    //    stransform.getOrigin().y():0.047869
    //    stransform.getOrigin().z():0.118485


    printf("position_kinect.x()%f\n", position_kinect.x());
    printf("position_kinect.y()%f\n", position_kinect.y());
    printf("position_kinect.z()%f\n", position_kinect.z());
    printf("dir_kinect.x()%f\n", dir_kinect.x());
    printf("dir_kinect.y()%f\n", dir_kinect.y());
    printf("dir_kinect.z()%f\n", dir_kinect.z());

    //    tf::Quaternion quaternion0;
    //    quaternion0.setX(-0.606064);
    //    quaternion0.setY(0.487114);
    //    quaternion0.setZ(-0.355795);
    //    quaternion0.setW(0.518476);

    tf::Quaternion quaternion0;
    tf::Vector3 angle0(-PI/2.0-PI/10.0, 0, -PI/2.0-PI/72.0);
    Eulerangles2Quaternion(angle0, quaternion0);


    //tf::Vector3 translation0(-0.00336185, 0.047869, 0.118485);
    tf::Vector3 translation0(-0.00336185, 0.047869, 0.075485);

    tf::Quaternion quaternion1;
    tf::Vector3 angle1(0, 0, 0);
    Eulerangles2Quaternion(angle1, quaternion1);

    tf::Quaternion quaternion2;
    tf::Vector3 angle2(0, 0, 0);
    Eulerangles2Quaternion(angle2, quaternion2);

    tf::Vector3 translation1(position_kinect.x(), position_kinect.y(), position_kinect.z());
    tf::Vector3 translation2(dir_kinect.x(), dir_kinect.y(), dir_kinect.z());

    tf::TransformBroadcaster br0;
    tf::Transform transform0;
    tf::TransformBroadcaster br1;
    tf::Transform transform1;
    tf::TransformBroadcaster br2;
    tf::Transform transform2;

    std::stringstream stt0;
    stt0 << "kinect_frame";

    std::stringstream stt1;
    stt1 << "touch_point";

    std::stringstream stt2;
    stt2 << "touch_dir";

    std::string frame1="r_gripper_tool_frame";
    std::string frame2=stt0.str();
    std::string frame4=stt1.str();
    std::string frame5=stt2.str();

    while(n.ok()){
        ros::Time time=ros::Time::now();

        transform0.setOrigin(translation0);
        transform0.setRotation(quaternion0);
        br0.sendTransform(tf::StampedTransform(transform0, time, frame1, frame2));

        transform1.setOrigin(translation1);
        transform1.setRotation(quaternion1);
        br1.sendTransform(tf::StampedTransform(transform1, time, frame2, frame4));

        transform2.setOrigin(translation2);
        transform2.setRotation(quaternion2);
        br2.sendTransform(tf::StampedTransform(transform2, time, frame2, frame5));

        ros::Duration(1.0).sleep();
    }
}

//get left gripper touch point
void get_l_touch_point_and_dir(ros::NodeHandle &n, const tf::Vector3 &position_kinect, const tf::Vector3 &dir_kinect, tf::Vector3 &position_base, tf::Vector3 &dir_base){

    tf::Quaternion quaternion0;
    tf::Vector3 angle0(-PI/2.0-PI/10.0, 0, -PI/2.0-PI/72.0);
    Eulerangles2Quaternion(angle0, quaternion0);

    tf::Quaternion quaternion1;
    tf::Vector3 angle1(0, 0, 0);
    Eulerangles2Quaternion(angle1, quaternion1);

    tf::Quaternion quaternion2;
    tf::Vector3 angle2(0, 0, 0);
    Eulerangles2Quaternion(angle2, quaternion2);

    tf::Vector3 translation0(-0.00336185, 0.047869, 0.075485);
    tf::Vector3 translation1(position_kinect.x(), position_kinect.y(), position_kinect.z());
    tf::Vector3 translation2(dir_kinect.x(), dir_kinect.y(), dir_kinect.z());

    tf::TransformBroadcaster br0;
    tf::Transform transform0;
    tf::TransformBroadcaster br1;
    tf::Transform transform1;
    tf::TransformBroadcaster br2;
    tf::Transform transform2;

    std::stringstream stt0;
    stt0 << "kinect_frame";

    std::stringstream stt1;
    stt1 << "touch_point";

    std::stringstream stt2;
    stt2 << "touch_dir";

    std::string frame1="r_gripper_tool_frame";
    std::string frame2=stt0.str();
    std::string frame3="base_link";
    std::string frame4=stt1.str();
    std::string frame5=stt2.str();

    tf::TransformListener listener0;
    tf::StampedTransform stransform0;
    tf::TransformListener listener1;
    tf::StampedTransform stransform1;

    int flag=0;

    while(!flag){
        ros::Time time=ros::Time::now();
        if (n.ok()){
            transform0.setOrigin(translation0);
            transform0.setRotation(quaternion0);
            br0.sendTransform(tf::StampedTransform(transform0, time, frame1, frame2));

            transform1.setOrigin(translation1);
            transform1.setRotation(quaternion1);
            br1.sendTransform(tf::StampedTransform(transform1, time, frame2, frame4));

            transform2.setOrigin(translation2);
            transform2.setRotation(quaternion2);
            br2.sendTransform(tf::StampedTransform(transform2, time, frame2, frame5));
        }
        else{
            std::cerr<<"node is not ok!"<<std::endl;
            return;
        }

        try{
            listener0.lookupTransform(frame3,frame4,
                                      ros::Time(0), stransform0);
            listener1.lookupTransform(frame3,frame5,
                                      ros::Time(0), stransform1);
            flag=1;
        }
        catch (tf::TransformException ex){
            // cerr<<ex.what()<<endl;
        }
    }

    printf("stransform0.getOrigin().x()%f\n", stransform0.getOrigin().x());
    printf("stransform0.getOrigin().y()%f\n", stransform0.getOrigin().y());
    printf("stransform0.getOrigin().z()%f\n", stransform0.getOrigin().z());
    printf("stransform1.getOrigin().x()%f\n", stransform1.getOrigin().x());
    printf("stransform1.getOrigin().y()%f\n", stransform1.getOrigin().y());
    printf("stransform1.getOrigin().z()%f\n", stransform1.getOrigin().z());

    position_base.setX(stransform0.getOrigin().x());
    position_base.setY(stransform0.getOrigin().y());
    position_base.setZ(stransform0.getOrigin().z());

    dir_base.setX(stransform1.getOrigin().x());
    dir_base.setY(stransform1.getOrigin().y());
    dir_base.setZ(stransform1.getOrigin().z());

    printf("position_base.x()%f\n", position_base.x());
    printf("position_base.y()%f\n", position_base.y());
    printf("position_base.z()%f\n", position_base.z());
    printf("dir_base.x()%f\n", dir_base.x());
    printf("dir_base.y()%f\n", dir_base.y());
    printf("dir_base.z()%f\n", dir_base.z());
}

//get right gripper touch point
void get_r_touch_point_and_dir(ros::NodeHandle &n, const tf::Vector3 &position_kinect, const tf::Vector3 &dir_kinect, tf::Vector3 &position_base, tf::Vector3 &dir_base){

    tf::Quaternion quaternion0;
    tf::Vector3 angle0(-PI/2.0-PI/10.0, 0, -PI/2.0-PI/72.0);
    Eulerangles2Quaternion(angle0, quaternion0);

    tf::Quaternion quaternion1;
    tf::Vector3 angle1(0, 0, 0);
    Eulerangles2Quaternion(angle1, quaternion1);

    tf::Quaternion quaternion2;
    tf::Vector3 angle2(0, 0, 0);
    Eulerangles2Quaternion(angle2, quaternion2);

    tf::Vector3 translation0(-0.00336185, 0.047869, 0.075485);
    tf::Vector3 translation1(position_kinect.x(), position_kinect.y(), position_kinect.z());
    tf::Vector3 translation2(dir_kinect.x(), dir_kinect.y(), dir_kinect.z());

    tf::TransformBroadcaster br0;
    tf::Transform transform0;
    tf::TransformBroadcaster br1;
    tf::Transform transform1;
    tf::TransformBroadcaster br2;
    tf::Transform transform2;

    std::stringstream stt0;
    stt0 << "kinect_frame";

    std::stringstream stt1;
    stt1 << "touch_point";

    std::stringstream stt2;
    stt2 << "touch_dir";

    std::string frame1="l_gripper_tool_frame";
    std::string frame2=stt0.str();
    std::string frame3="base_link";
    std::string frame4=stt1.str();
    std::string frame5=stt2.str();

    tf::TransformListener listener0;
    tf::StampedTransform stransform0;
    tf::TransformListener listener1;
    tf::StampedTransform stransform1;

    int flag=0;

    while(!flag){
        ros::Time time=ros::Time::now();
        if (n.ok()){
            transform0.setOrigin(translation0);
            transform0.setRotation(quaternion0);
            br0.sendTransform(tf::StampedTransform(transform0, time, frame1, frame2));

            transform1.setOrigin(translation1);
            transform1.setRotation(quaternion1);
            br1.sendTransform(tf::StampedTransform(transform1, time, frame2, frame4));

            transform2.setOrigin(translation2);
            transform2.setRotation(quaternion2);
            br2.sendTransform(tf::StampedTransform(transform2, time, frame2, frame5));
        }
        else{
            std::cerr<<"node is not ok!"<<std::endl;
            return;
        }

        try{
            listener0.lookupTransform(frame3,frame4,
                                      ros::Time(0), stransform0);
            listener1.lookupTransform(frame3,frame5,
                                      ros::Time(0), stransform1);
            flag=1;
        }
        catch (tf::TransformException ex){
            // cerr<<ex.what()<<endl;
        }
    }

    position_base.setX(stransform0.getOrigin().x());
    position_base.setX(stransform0.getOrigin().y());
    position_base.setZ(stransform0.getOrigin().z());

    dir_base.setX(stransform1.getOrigin().x());
    dir_base.setX(stransform1.getOrigin().y());
    dir_base.setZ(0);
}
