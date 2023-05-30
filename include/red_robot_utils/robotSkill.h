#pragma once
#include <ros/ros.h>
#include <franka_ros_example/frankX_move.h>
#include <franka_ros_example/frankX_gripper.h>
#include <franka_ros_example/frankX_joint.h>
#include <franka_ros_example/frankX_pickup.h>
#include <franka_ros_example/frankX_grasp.h>



class RobotSkill
{
public:

    RobotSkill(ros::NodeHandle& n);

    ros::ServiceClient move_client;
    ros::ServiceClient gripper_client;
    ros::ServiceClient joint_client;
    ros::ServiceClient grasp_client;

protected:

    ros::NodeHandle n;

};

class RealRobotSkill : public RobotSkill
{
public:
    RealRobotSkill(ros::NodeHandle& n);
private:
};

class SimulatedRobotSkill : public RobotSkill
{
public:
    SimulatedRobotSkill(ros::NodeHandle& n);
};