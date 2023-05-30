#pragma once
#include <ros/ros.h>
#include "franka_ros_example/frankX_grasp.h"
#include "franka_ros_example/frankX_gripper.h"
#include "franka_ros_example/frankX_joint.h"
#include "franka_ros_example/frankX_move.h"
#include "dynamic_reconfigure/server.h"
#include "robot_controllers/RobotConfig.h"

struct Pose
{
    double x;
    double y;
    double z;
};

struct RobotParams
{
    bool is_real;
    float arm_speed;
    float gripper_width;
    float gripper_speed;

    float elbow;
    int force_limit;
    
    float a;
    float b;
    float c;

    float x;
    float y;
    float z;
};

/**
 * class used to perform basic action with the robot
*/
class Controller
{
public:

    Controller(ros::NodeHandle& n);
    ~Controller();
    void activateDynamicServer();
    /**
     * @return true if the movement is completed correctly 100%
    */
    bool moveCartesian();
    void moveHome();
    /**
     * parameters used are taken from robotParams
     * @return width of the gripper after the action
    */
    double moveGripper();
    /**
     * parameters used are taken from robotParams
     * @return width of the gripper after the action
    */
    double clampGripper();
    /**
     * parameters used are taken from robotParams
     * @return width of the gripper after the action
    */
    double releaseGripper();
    
    

protected:

    /**
     * change service client depending on the value of robotParams.is_real
    */
    void setServices();
    
    /**
     * values of this data structure are updated from the reconfiguration GUI
    */
    RobotParams robotParams;

    // low level services used to give commands to the robot, service servers associated could be
    // real or simulated robot
    ros::ServiceClient move_client;
    ros::ServiceClient gripper_client;
    ros::ServiceClient joint_client;
    ros::ServiceClient grasp_client;
    
    ros::NodeHandle& n;

    // objects and functions used for the dynamic reconfiguration server
    void reconfigure_robot_params(robot_controllers::RobotConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robot_controllers::RobotConfig>* robot_param_server;
    dynamic_reconfigure::Server<robot_controllers::RobotConfig>::CallbackType robot_param_callback;
    

};