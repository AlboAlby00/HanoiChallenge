#include "robot_controllers/controller/controller.h"


Controller::Controller(ros::NodeHandle& n) 
    : n(n)
{
    activateDynamicServer();
    setServices();
}

Controller::~Controller()
{
    free(this->robot_param_server);
}

void Controller::activateDynamicServer()
{
    //setup dynamic_reconfigure
    this->robot_param_server = 
        new dynamic_reconfigure::Server<robot_controllers::RobotConfig>(n); 
    this->robot_param_callback = boost::bind(&Controller::reconfigure_robot_params, this, _1, _2);
    this->robot_param_server->setCallback(this->robot_param_callback);

}

/*
    function called everytime a parameter is modified from the reconfiguration gui
*/
void Controller::reconfigure_robot_params(robot_controllers::RobotConfig &config, uint32_t level) 
{
    //ROS_INFO("setting arm speed to: %f",config.arm_speed);
    this->robotParams.arm_speed=config.arm_speed;

    //ROS_INFO( (config.is_real ? "using real robot" : "using simulation") );
    this->robotParams.is_real = config.is_real;

    //ROS_INFO("setting gripper width to: %f",config.gripper_width);
    this->robotParams.gripper_width = config.gripper_width;
    this->robotParams.gripper_speed = config.gripper_speed;

    this->robotParams.a = config.a;
    this->robotParams.b = config.b;
    this->robotParams.c = config.c;

    this->robotParams.x = config.x;
    this->robotParams.y = config.y;
    this->robotParams.z = config.z;

    this->robotParams.elbow = config.elbow;
    this->robotParams.force_limit = config.force_limit;

    ROS_INFO("reconfiguring robot params");

    setServices();
}

/*
    read value is_real from params, and create the service clients accordingly
*/
void Controller::setServices()
{
    std::string source = robotParams.is_real ? "frankX_service_node" : "simulated_robot_node";
    
    this->move_client = 
        n.serviceClient<franka_ros_example::frankX_move>("/"+source+"/frankX_move");
    this->gripper_client = 
        n.serviceClient<franka_ros_example::frankX_gripper>("/"+source+"/frankX_gripper");
    this->joint_client = 
        n.serviceClient<franka_ros_example::frankX_joint>("/"+source+"/frankX_joint");
    this->grasp_client = 
        n.serviceClient<franka_ros_example::frankX_grasp>("/"+source+"/frankX_grasp");
    
}

/*
    move robotic arm to Home position
*/
void Controller::moveHome()
{
    franka_ros_example::frankX_joint arguments;
    arguments.request.q0 = 0;
    arguments.request.q1 = -0.785398163;
    arguments.request.q2 = 0;
    arguments.request.q3 = -2.35619449;
    arguments.request.q4 = 0;
    arguments.request.q5 = 1.57079632679;
    arguments.request.q6 = 0.785398163397;
    arguments.request.speed = this->robotParams.arm_speed;

    this->joint_client.call(arguments);

}

bool Controller::moveCartesian()
{
    franka_ros_example::frankX_move pose_arguments;
    pose_arguments.request.x = robotParams.x;
    pose_arguments.request.y = robotParams.y;
    pose_arguments.request.z = robotParams.z;
    pose_arguments.request.a = robotParams.a;
    pose_arguments.request.b = robotParams.b;
    pose_arguments.request.c = robotParams.c;
    pose_arguments.request.force_limit = robotParams.force_limit;
    pose_arguments.request.speed = robotParams.arm_speed;
    pose_arguments.request.elbow = robotParams.elbow;
    bool result = this->move_client.call(pose_arguments);
    return result;

}

/*
    read width from params and open the gripper
*/
double Controller::moveGripper()
{
    franka_ros_example::frankX_gripper move_gripper_arguments;
    move_gripper_arguments.request.speed = robotParams.gripper_speed;
    move_gripper_arguments.request.option = "move";
    move_gripper_arguments.request.move_width = robotParams.gripper_width;
    this->gripper_client.call(move_gripper_arguments);
    ROS_INFO("width of object: %f", move_gripper_arguments.response.width_after);
    return move_gripper_arguments.response.width_after;
}

/*
    read width from params and clamp the gripper
*/
double Controller::clampGripper()
{
    franka_ros_example::frankX_gripper clamp_gripper_arguments;
    clamp_gripper_arguments.request.speed = robotParams.gripper_speed;
    clamp_gripper_arguments.request.option = "clamp";
    clamp_gripper_arguments.request.clamp_width = 0;
    this->gripper_client.call(clamp_gripper_arguments);
    ROS_INFO("width of object: %f", clamp_gripper_arguments.response.width_after);
    return clamp_gripper_arguments.response.width_after;
}

/*
    read width from params and release the gripper
*/
double Controller::releaseGripper()
{
    franka_ros_example::frankX_gripper release_gripper_arguments;
    release_gripper_arguments.request.speed = robotParams.gripper_speed;
    release_gripper_arguments.request.option = "release";
    release_gripper_arguments.request.release_width = robotParams.gripper_width;
    this->gripper_client.call(release_gripper_arguments);
    ROS_INFO("width of object: %f", release_gripper_arguments.response.width_after);
    return release_gripper_arguments.response.width_after;
}


