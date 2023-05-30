#include "robot_controllers/pickAndPlaceController/pickAndPlaceController.h"

PickAndPlaceController::PickAndPlaceController(
    ros::NodeHandle& n_pick, ros::NodeHandle& n_controller) : n_pick(n_pick),Controller(n_controller)
{
    this->pick_and_place_param_server = 
        new dynamic_reconfigure::Server<robot_controllers::PickAndPlaceConfig>(n_pick);

    ros::service::waitForService("/computer_vision/get_block_position", ros::Duration(3.0));
    this->brickDetectionService = 
        n.serviceClient<red_msgs::WorldPoint>("/computer_vision/get_block_position");
        

    activateDynamicServer();
}

PickAndPlaceController::~PickAndPlaceController()
{
    free(pick_and_place_param_server);
}

void PickAndPlaceController::activateDynamicServer()
{
    
    //setup dynamic_reconfigure
    

    this->pick_and_place_param_callback = 
        boost::bind(&PickAndPlaceController::reconfigure_pick_and_place_params, this, _1, _2);
    this->pick_and_place_param_server->setCallback(this->pick_and_place_param_callback);
    
}

void PickAndPlaceController::reconfigure_pick_and_place_params(
    robot_controllers::PickAndPlaceConfig &config, uint32_t level)
{
    this->pickAndPlaceParams.delta_height = config.delta_height;

    this->pickAndPlaceParams.x_pick = config.x_pick;
    this->pickAndPlaceParams.y_pick = config.y_pick;
    this->pickAndPlaceParams.z_pick = config.z_pick;

    this->pickAndPlaceParams.x_place = config.x_place;
    this->pickAndPlaceParams.y_place = config.y_place;
    this->pickAndPlaceParams.z_place = config.z_place;

    ROS_INFO("reconfiguring pick and place params");
}

/*
    pick an object, returns the width of the object
*/
double PickAndPlaceController::pick()
{
    this->robotParams.gripper_width = 0.08;
    this->moveGripper();

    this->robotParams.x = this->pickAndPlaceParams.x_pick;
    this->robotParams.y = this->pickAndPlaceParams.y_pick;
    this->robotParams.z = this->pickAndPlaceParams.z_pick + this->pickAndPlaceParams.delta_height;
    this->moveCartesian();

    this->robotParams.x = this->pickAndPlaceParams.x_pick;
    this->robotParams.y = this->pickAndPlaceParams.y_pick;
    this->robotParams.z = this->pickAndPlaceParams.z_pick;
    this->moveCartesian();

    
    double width = this->clampGripper();

    this->robotParams.x = this->pickAndPlaceParams.x_pick;
    this->robotParams.y = this->pickAndPlaceParams.y_pick;
    this->robotParams.z = this->pickAndPlaceParams.z_pick + this->pickAndPlaceParams.delta_height;
    this->moveCartesian();
    
    return width;

}

/*
    Place an object in the location set in the pickAndPlaceParams
*/

bool PickAndPlaceController::place()
{

    // move to place up position
    this->robotParams.x = this->pickAndPlaceParams.x_place;
    this->robotParams.y = this->pickAndPlaceParams.y_place;
    this->robotParams.z = this->pickAndPlaceParams.z_place + this->pickAndPlaceParams.delta_height;
    this->moveCartesian();

    // move to place position
    this->robotParams.x = this->pickAndPlaceParams.x_place;
    this->robotParams.y = this->pickAndPlaceParams.y_place;
    this->robotParams.z = this->pickAndPlaceParams.z_place;
    this->moveCartesian();

    // release gripper
    this->robotParams.gripper_width = 0.08;
    this->moveGripper();

    // move to place position
    this->robotParams.x = this->pickAndPlaceParams.x_place;
    this->robotParams.y = this->pickAndPlaceParams.y_place;
    this->robotParams.z = this->pickAndPlaceParams.z_place;
    this->moveCartesian();

    return true;

}

/*
    tries to pick an object, if picked, place it, otherwise it goes to home position
*/
bool PickAndPlaceController::pickAndPlace()
{
    double gripper_width = this->pick();
    
    if(gripper_width>0.001)
    {
        this->place();
        return true;
    }
    else
    {
        this->moveHome();
        return false;
    }
}

bool PickAndPlaceController::scanAndPickAndPlace()
{
    red_msgs::WorldPoint serviceArgs;
    this->brickDetectionService.call(serviceArgs);

    double x_pick = serviceArgs.response.x;
    double y_pick = serviceArgs.response.y;
    bool result;

    bool block_detected = ( x_pick!=0 && y_pick!=0);
    if(block_detected)
    {
        this->pickAndPlaceParams.x_pick = serviceArgs.response.x;
        this->pickAndPlaceParams.y_pick = serviceArgs.response.y;
        ROS_INFO("X scanned: %f, Y scanned: %f, Z scanned: %f", 
            serviceArgs.response.x, serviceArgs.response.y, serviceArgs.response.z);
        result = this->pickAndPlace();
    }
    else
    {
       ROS_INFO(" no block scanned ");
       result = false; 
    }

    return result;
}