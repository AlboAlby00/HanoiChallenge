#include "red_robot_utils/robotSkill.h"

RobotSkill::RobotSkill(ros::NodeHandle& n) : n(n)
{
   
}

RealRobotSkill::RealRobotSkill(ros::NodeHandle& n) : RobotSkill(n)
{
    this->move_client = 
        n.serviceClient<franka_ros_example::frankX_move>("/frankX_service_node/frankX_move");
    this->gripper_client = 
        n.serviceClient<franka_ros_example::frankX_gripper>("/frankX_service_node/frankX_gripper");
    this->joint_client = 
        n.serviceClient<franka_ros_example::frankX_joint>("/frankX_service_node/frankX_joint");
    this->grasp_client = 
        n.serviceClient<franka_ros_example::frankX_grasp>("/frankX_service_node/frankX_grasp");
}

SimulatedRobotSkill::SimulatedRobotSkill(ros::NodeHandle& n) : RobotSkill(n)
{
    this->move_client = 
        n.serviceClient<franka_ros_example::frankX_move>("/simulated_robot_node/frankX_move");
    this->gripper_client = 
        n.serviceClient<franka_ros_example::frankX_gripper>("/simulated_robot_node/frankX_gripper");
    this->joint_client = 
        n.serviceClient<franka_ros_example::frankX_joint>("/simulated_robot_node/frankX_joint");
    this->grasp_client = 
        n.serviceClient<franka_ros_example::frankX_grasp>("/simulated_robot_node/frankX_grasp");

}


