#pragma once
#include <ros/ros.h>
#include "franka_skill/skill.h"
#include <franka_ros_example/frankX_move.h>
#include <franka_ros_example/frankX_gripper.h>
#include <franka_ros_example/frankX_joint.h>
#include <franka_ros_example/frankX_grasp.h>

typedef franka_ros_example::frankX_move::Request moveRequest;
typedef franka_ros_example::frankX_move::Response moveResponse;
typedef franka_ros_example::frankX_gripper::Request gripperRequest;
typedef franka_ros_example::frankX_gripper::Response gripperResponse;
typedef franka_ros_example::frankX_joint::Request jointRequest;
typedef franka_ros_example::frankX_joint::Response jointResponse;
typedef franka_ros_example::frankX_grasp::Request graspRequest;
typedef franka_ros_example::frankX_grasp::Response graspResponse;

class SimulatedRobot 
{
    public:
        SimulatedRobot(ros::Rate loop_rate_, ros::NodeHandle& n);
        bool service_move_call(moveRequest  &req, moveResponse &res);
        bool service_gripper_call(gripperRequest& req, gripperResponse& res);
        bool service_joint_call(jointRequest  &req, jointResponse &res);
        bool service_grasp_call(graspRequest& req, graspResponse& res);

    private:
        Eigen::VectorXd* get_current_joint_pose();
        void sendToRobot(const MotionData& data);
        void sendToRobot(const moveit::planning_interface::MoveGroupInterface::Plan &myPlan);

        ros::NodeHandle n;
        ros::Publisher jointPub;
        ros::Rate loop_rate;
        TrajectoryGeneratorInterface trajectoryInterface;

};
