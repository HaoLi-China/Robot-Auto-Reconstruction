#include "ros/ros.h"
#include "std_msgs/String.h"
#include "auto_reconstruction.h"

#include <sstream>

#include <stdio.h> 
#include <stdlib.h> 
#include <errno.h> 
#include <string.h> 
#include <sys/types.h> 
#include <netinet/in.h> 
#include <sys/socket.h> 
#include <sys/wait.h> 
#include <pthread.h>

#include <tf/transform_listener.h>

using namespace std;

#define SERVPORT 3333 /*服务器监听端口号 */ 
#define BACKLOG 10 /* 最大同时连接请求数 */ 

int sockfd,client_fd,sin_size; /*sock_fd:监听 socket;client_fd:数据传输 socket */
struct sockaddr_in my_addr; /* 本机地址信息 */ 
struct sockaddr_in remote_addr; /* 客户端地址信息 */ 

const int MAXRECV = 10240;
int stopped=0;

void *thread(void *ptr)
{
    while(1){
        ros::spinOnce();
        ros::Duration(0,100000).sleep();
    }
    return 0;
}

//结束循环
void stop()
{
    close(client_fd);
    stopped = 1;
}

//播放声音提示
void play_sound_tip(char ch){
    //a:init robot pose
    //b:init left arm pose
    //c:init right arm pose
    //d:make left scanner up and down
    //e:make right scanner up and down
    //f:l_arm pick up kinect
    //g:l_arm put down kinect
    //h:r_arm pick up kinect
    //i:r_arm put down kinect
    //j:left arm push object
    //k:right arm push object
    //l:set head pose
    //m:left arm take back
    //n:right arm take back
    //o:get left gripper touch point
    //p:get right gripper touch point
    //q:test calibration result
    //z:close

    switch(ch){
    case 'a':
        system("rosrun sound_play say.py \"I will change my pose.\"");
        printf("I will change my pose.\n");
        break;
    case 'b':
        system("rosrun sound_play say.py \"I will change my left arm pose.\"");
        printf("I will change my left arm pose.\n");
        break;
    case 'c':
        system("rosrun sound_play say.py \"I will change my right arm pose.\"");
        printf("I will change my right arm pose.\n");
        break;
    case 'd':
        system("rosrun sound_play say.py \"I will make scanner up and down.\"");
        printf("I will make scanner up and down.\n");
        break;
    case 'e':
        system("rosrun sound_play say.py \"I will make scanner up and down.\"");
        printf("I will make scanner up and down.\n");
        break;
    case 'f':
        system("rosrun sound_play say.py \"I will pick up kinect.\"");
        printf("I will pick up kinect.\n");
        break;
    case 'g':
        system("rosrun sound_play say.py \"I will put down kinect.\"");
        printf("I will put down kinect.\n");
        break;
    case 'h':
        system("rosrun sound_play say.py \"I will pick up kinect.\"");
        printf("I will pick up kinect.\n");
        break;
    case 'i':
        system("rosrun sound_play say.py \"I will put down kinect.\"");
        printf("I will put down kinect.\n");
        break;
    case 'j':
        system("rosrun sound_play say.py \"I will push the object.\"");
        printf("I will push the object.\n");
        break;
    case 'k':
        system("rosrun sound_play say.py \"I will push the object.\"");
        printf("I will push the object.\n");
        break;
    case 'l':
        system("rosrun sound_play say.py \"I will set head pose.\"");
        printf("I will set head pose.\n");
        break;
    case 'm':
        system("rosrun sound_play say.py \"I will take back the arm.\"");
        printf("I will take back the arm.\n");
        break;
    case 'n':
        system("rosrun sound_play say.py \"I will take back the arm.\"");
        printf("I will take back the arm.\n");
        break;
    case 'o':
        system("rosrun sound_play say.py \"I will find gripper touch point.\"");
        printf("I will find gripper touch point.\n");
        break;
    case 'p':
        system("rosrun sound_play say.py \"I will find gripper touch point.\"");
        printf("I will find gripper touch point.\n");
        break;
    case 'q':
        system("rosrun sound_play say.py \"I will test the calibration result.\"");
        printf("I will test the calibration result.\n");
    case 'z':
        system("rosrun sound_play say.py \"The connection will be closed.\"");
        printf("The connection will be closed.\n");
        break;
    default:
        system("rosrun sound_play say.py \"Wrong command.\"");
        printf("Wrong command.\n");
        break;
    }
}

//接收数据
int recvData(char buf[])
{
    memset ( buf, 0, MAXRECV + 1 );

    int status = recv(client_fd, buf, MAXRECV, 0);

    if ( status == -1 )
    {
        printf("status == -1 errno == %s in Socket::recv\n",errno);
        return 0;
    }
    else if( status == 0 )
    {
        stop();
        return 0;
    }
    else
    {
        return status;
    }
}

//向指定客户端发送数据
bool sendData(char *ch,int len)
{
    int status = send(client_fd, ch, len, 0);
    if ( status == -1 )
    {
        return false;
    }
    else
    {
        return true;
    }
}

//string to int
int str2int(char* ch)
{
    return atoi(ch);
}

//string to float
float str2float(char* ch)
{
    return atof(ch);
}

void data_parse_a(char* ch, double *pos_left_arm, double *pos_right_arm, tf::Vector3 &head_focus, float &torso_up){
    int index=0;
    int count=0;
    char tem[20];

    while(*ch!='\0'){
        if(*ch==',' || *ch==';'){
            strncpy(tem,ch-index,index);
            tem[index]='\0';
            if(count < 7){
                pos_left_arm[count]=str2float(tem);
            }
            else if(count < 14){
                pos_right_arm[count-7]=str2float(tem);
            }
            else if(count < 17){
                head_focus[count-14]=str2float(tem);
            }
            else{
                torso_up=str2float(tem);
            }
            index=0;
            count++;
            ch++;
            cout<<tem<<endl;
            continue;
        }
        index++;
        ch++;
    }
}

void data_parse_bc(char* ch, double *pos_left_arm){
    int index=0;
    int count=0;
    char tem[20];

    while(*ch!='\0'){
        if(*ch==',' || *ch==';'){
            strncpy(tem,ch-index,index);
            tem[index]='\0';
            pos_left_arm[count]=str2float(tem);
            index=0;
            count++;
            ch++;
            cout<<tem<<endl;
            continue;
        }
        index++;
        ch++;
    }
}

void data_parse_de(char* ch, float &down_value, float &up_value){
    int index=0;
    int count=0;
    char tem[20];

    while(*ch!='\0'){
        if(*ch==',' || *ch==';'){
            strncpy(tem,ch-index,index);
            tem[index]='\0';
            if(count==0){
                down_value=str2float(tem);
            }
            else{
                up_value=str2float(tem);
            }
            index=0;
            count++;
            ch++;
            cout<<tem<<endl;
            continue;
        }
        index++;
        ch++;
    }
}

void data_parse_jkmn(char* ch, tf::Vector3& position, tf::Vector3& direction){
    int index=0;
    int count=0;
    char tem[20];

    while(*ch!='\0'){
        if(*ch==',' || *ch==';'){
            strncpy(tem,ch-index,index);
            tem[index]='\0';
            if(count<3){
                position[count]=str2float(tem);
            }
            else{
                direction[count-3]=str2float(tem);
            }
            index=0;
            count++;
            ch++;
            cout<<tem<<endl;
            continue;
        }
        index++;
        ch++;
    }
}

void data_parse_l(char* ch, tf::Vector3 &pos){
    int index=0;
    int count=0;
    char tem[20];

    while(*ch!='\0'){
        if(*ch==',' || *ch==';'){
            strncpy(tem,ch-index,index);
            tem[index]='\0';
            pos[count]=str2float(tem);
            index=0;
            count++;
            ch++;
            cout<<tem<<endl;
            continue;
        }
        index++;
        ch++;
    }
}

void data_parse_op(char* ch, tf::Vector3& position, tf::Vector3& direction){
    int index=0;
    int count=0;
    char tem[20];

    while(*ch!='\0'){
        if(*ch==',' || *ch==';'){
            strncpy(tem,ch-index,index);
            tem[index]='\0';
            if(count<3){
                position[count]=str2float(tem);
            }
            else{
                direction[count-3]=str2float(tem);
            }
            index=0;
            count++;
            ch++;
            cout<<tem<<endl;
            continue;
        }
        index++;
        ch++;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "socketServer");
    ros::NodeHandle n;

    ros::Rate loop_rate(10);

    //Create robot controller interface
    simple_robot_control::Robot robot;
    PR2MoveArmJoint move_arm_joint;

    //    pthread_t id;
    //    int ret = pthread_create(&id, NULL, thread, NULL);
    //    if(ret) {
    //        printf("Create pthread error!\n");
    //    }
    //pthread_join(id, NULL);

    if ((sockfd = socket(AF_INET, SOCK_STREAM, 0)) == -1)//建立 socket
    {
        perror("socket");
        exit(1);
    }

    my_addr.sin_family=AF_INET;
    my_addr.sin_port=htons(SERVPORT);
    my_addr.sin_addr.s_addr = INADDR_ANY; //表示监听任何地址
    bzero(&(my_addr.sin_zero),8);

    if (bind(sockfd, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) == -1) //将本机地址与建立的套接字号进行绑定
    {
        perror("bind");
        exit(1);
    }

    if (listen(sockfd, BACKLOG) == -1) //开始监听
    {
        perror("listen");
        exit(1);
    }

    while (ros::ok())
    {
        sin_size = sizeof(my_addr);

        ROS_INFO("%s", "waiting for a connection");

        if ((client_fd = accept(sockfd, (struct sockaddr*)&remote_addr, (socklen_t *) &sin_size)) == -1)//接收客户端的连接
        {
            perror("accept");
            continue;
        }
        ROS_INFO("%s", "received a connection");

        stopped=0;

        while(!stopped){

            char message [MAXRECV+1];

            if(recvData(message)){
                printf("=====%s=====\n", message);
                printf("%d\n", strlen(message));

                if(message!=NULL&&strlen(message)!=0){
                    //a:init robot pose
                    //b:init left arm pose
                    //c:init right arm pose
                    //d:make left scanner up and down
                    //e:make right scanner up and down
                    //f:l_arm pick up kinect
                    //g:l_arm put down kinect
                    //h:r_arm pick up kinect
                    //i:r_arm put down kinect
                    //j:left arm push object
                    //k:right arm push object
                    //l:set head pose
                    //m:left arm take back
                    //n:right arm take back
                    //o:get left gripper touch point
                    //p:get right gripper touch point
                    //q:test calibration result
                    //z:close

                    play_sound_tip(message[0]);

                    switch(message[0]){
                    case 'a':
                    {
                        double pos_left_arm[7];
                        double pos_right_arm[7];
                        tf::Vector3 head_focus;
                        float torso_up;
                        data_parse_a(message+2, pos_left_arm, pos_right_arm, head_focus, torso_up);
                        init_robot_pose(robot, pos_left_arm, pos_right_arm, head_focus, torso_up);
                        sendData("finished",9);
                        break;
                    }
                    case 'b':
                    {
                        double pos_left_arm[7];
                        data_parse_bc(message+2, pos_left_arm);
                        init_left_arm_pose(robot, pos_left_arm);
                        sendData("finished",9);
                        break;
                    }
                    case 'c':
                    {
                        double pos_right_arm[7];
                        data_parse_bc(message+2, pos_right_arm);
                        init_right_arm_pose(robot, pos_right_arm);
                        sendData("finished",9);
                        break;
                    }
                    case 'd':
                    {
                        float down_value=0;
                        float up_value=0;
                        data_parse_de(message+2, down_value, up_value);
                        up_down_left_scanner(move_arm_joint, down_value, up_value);
                        sendData("finished",9);
                        break;
                    }
                    case 'e':
                    {
                        float down_value=0;
                        float up_value=0;
                        data_parse_de(message+2, down_value, up_value);
                        up_down_right_scanner(move_arm_joint, down_value, up_value);
                        sendData("finished",9);
                        break;
                    }
                    case 'f':
                        l_pick_up_kinect(robot.left_arm, robot.left_gripper, robot.base);
                        sendData("finished",9);
                        break;
                    case 'g':
                        l_put_down_kinect(robot.left_arm, robot.left_gripper, robot.base);
                        sendData("finished",9);
                        break;
                    case 'h':
                        r_pick_up_kinect(robot.right_arm, robot.right_gripper, robot.base);
                        sendData("finished",9);
                        break;
                    case 'i':
                        r_put_down_kinect(robot.right_arm, robot.right_gripper, robot.base);
                        sendData("finished",9);
                        break;
                    case 'j':
                    {
                        tf::Vector3 position;
                        tf::Vector3 direction;
                        data_parse_jkmn(message+2, position, direction);
                        l_push_object(robot.left_arm, position, direction);
                        sendData("finished",9);
                        break;
                    }
                    case 'k':
                    {
                        tf::Vector3 position;
                        tf::Vector3 direction;
                        data_parse_jkmn(message+2, position, direction);
                        r_push_object(robot.right_arm, position, direction);
                        sendData("finished",9);
                        break;
                    }
                    case 'l':
                    {
                        tf::Vector3 head_focus;
                        data_parse_l(message+2, head_focus);
                        set_head_pose(robot.head, head_focus);
                        sendData("finished",9);
                        break;
                    }
                    case 'm':
                    {
                        tf::Vector3 position;
                        tf::Vector3 direction;
                        data_parse_jkmn(message+2, position, direction);
                        l_take_back(robot.left_arm, position, direction);
                        sendData("finished",9);
                        break;
                    }
                    case 'n':
                    {
                        tf::Vector3 position;
                        tf::Vector3 direction;
                        data_parse_jkmn(message+2, position, direction);
                        r_take_back(robot.right_arm, position, direction);
                        sendData("finished",9);
                        break;
                    }
                    case 'o':
                    {
                        tf::Vector3 position_kinect;
                        tf::Vector3 dir_kinect;
                        tf::Vector3 position_base;
                        tf::Vector3 dir_base;

                        data_parse_op(message+2, position_kinect, dir_kinect);
                        get_l_touch_point_and_dir(n, position_kinect, dir_kinect, position_base, dir_base);

                        printf("position_base.x()%f\n", position_base.x());
                        printf("position_base.y()%f\n", position_base.y());
                        printf("position_base.z()%f\n", position_base.z());
                        printf("dir_base.x()%f\n", dir_base.x());
                        printf("dir_base.y()%f\n", dir_base.y());
                        printf("dir_base.z()%f\n", dir_base.z());

                        std::stringstream stream;
                        stream << position_base.x() <<","<<position_base.y()<<","<<position_base.z()<<","
                               <<dir_base.x() <<","<<dir_base.y()<<","<<dir_base.z()<<";";
                        std::string str = stream.str();
                        char* ch_pos = const_cast<char*>(str.c_str());

                        printf("ch_pos:%s\n", ch_pos);

                        sendData(ch_pos,strlen(ch_pos)+1);
                        break;
                    }
                    case 'p':
                    {
                        tf::Vector3 position_kinect;
                        tf::Vector3 dir_kinect;
                        tf::Vector3 position_base;
                        tf::Vector3 dir_base;

                        data_parse_op(message+2, position_kinect, dir_kinect);
                        get_r_touch_point_and_dir(n, position_kinect, dir_kinect, position_base, dir_base);

                        std::stringstream stream;
                        stream << position_base.x() <<","<<position_base.y()<<","<<position_base.z()<<","<<dir_base.x() <<","<<dir_base.y()<<","<<dir_base.z()<<";";
                        std::string str = stream.str();
                        char* ch_pos = const_cast<char*>(str.c_str());

                        sendData(ch_pos,strlen(ch_pos)+1);
                        break;
                    }
                    case 'q'://just for test
                    {
                        tf::Vector3 position_kinect;
                        tf::Vector3 dir_kinect;

                        data_parse_op(message+2, position_kinect, dir_kinect);
                        test_calibration_result(n, position_kinect, dir_kinect);

                        sendData("finished",9);
                        break;
                    }
                    case 'z':
                    {
                        stop();
                        printf("The connection will be close.\n");
                        break;
                    }
                    default:
                        sendData("error msg",10);
                    }
                }
                else{
                    sendData("empty",6);
                }
            }
        }
        loop_rate.sleep();
    }

    return 0;
}
