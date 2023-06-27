/**
 * @file Mcu2MoveitNode.cpp
 * @author IKEMURA, Kei (ikemurakei2001@gmial.com)
 * @brief This node translates messages from the MCU to MoveIt! for trajectory calculation.
 * @version 0.1
 * @date 2023-06-27
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <ros/ros.h>

#include <robot_msgs/ArmStatus.h>
#include <sensor_msgs/JointState.h>
#include <rag2_moveit/CMDInfo.h>

static const float JOINT_VALUE_SCALAR = 1 / 16384.0;

static sensor_msgs::JointState jointState;
static rag2_moveit::CMDInfo cmdInfo;

static ros::Publisher cmdInfoPub;
static ros::Publisher jointStatePub;

void armStatusCb(const robot_msgs::ArmStatusConstPtr &msg)
{
    // -- set joint state message --
    jointState.header.stamp = ros::Time::now();
    jointState.header.frame_id = "";
    jointState.header.seq += 1;
    for (int i = 0; i < 6; i++)
        jointState.position[i] = msg->joint[i] * JOINT_VALUE_SCALAR;

    // -- publish joint state --
    jointStatePub.publish(jointState);

    // -- set command info --
    cmdInfo.header.stamp = ros::Time::now();
    cmdInfo.header.seq += 1;
    cmdInfo.mode = msg->mode;
    cmdInfo.orientation[3] = msg->target_orientation[0]; // w
    cmdInfo.orientation[0] = msg->target_orientation[1]; // i
    cmdInfo.orientation[1] = msg->target_orientation[2]; // j
    cmdInfo.orientation[2] = msg->target_orientation[3]; // k
    for (int i = 0; i < 3; i++)
        cmdInfo.positions[i] = msg->target_position[i];

    // -- pubish command info --
    cmdInfoPub.publish(cmdInfo);
}

int main(int ac, char **av)
{
    ros::init(ac, av, "mcu_to_moveit_node");
    ros::Time::init();

    ros::NodeHandle nh;

    // -- publisher --
    jointStatePub = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);
    cmdInfoPub = nh.advertise<rag2_moveit::CMDInfo>("/cmd_pub", 10);

    // -- subscriber --
    ros::Subscriber armCmdSub = nh.subscribe<robot_msgs::ArmStatus>("/arm_status", 10, armStatusCb);

    // -- initialize joint state message, can save time later --
    jointState.header.seq = 0;
    jointState.name.resize(6);
    jointState.position.resize(6);
    for (int i = 0; i < 6; i++)
    {
        jointState.position[i] = 0;
        jointState.name[i] = std::string("Joint_") + std::to_string(i + 1);
    }

    // -- intiailize command info message --
    cmdInfo.header.seq = 0;
    cmdInfo.header.frame_id = "";
    cmdInfo.name = "bocchi";
    cmdInfo.mode = 0;

    static ros::Rate rate(2000);
    while (ros::ok())
    {
        rate.sleep();
        ros::spinOnce();
    }
}