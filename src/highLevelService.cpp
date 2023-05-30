#include "red_robot_utils/highLevelService.h"


HighLevelService::HighLevelService(ros::NodeHandle& n, RobotSkill robotSkill) : n(n),robotSkill(robotSkill)
{
    this->moveHomeService = 
        this->n.advertiseService("move_home", 
            &HighLevelService::moveHomeCallback, this);

    this->moveToGroundService =         
        this->n.advertiseService("move_to_ground", 
            &HighLevelService::moveToGroundCallback, this);

    this->scanWorkspaceService =         
        this->n.advertiseService("scan_workspace", 
            &HighLevelService::scanWorkspaceCallback, this);

    this->pickAndPlaceService = 
        this->n.advertiseService("pick_and_place", 
            &HighLevelService::pickAndPlaceCallback, this);

    this->scanOneObjectToPickAndPlaceService = 
        this->n.advertiseService("scan_one_block_to_pick_and_place", 
            &HighLevelService::scanOneBlockToPickAndPlaceCallback, this);
}

bool HighLevelService::moveHomeCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->robotSkill.moveHome();
    return true;
}

bool HighLevelService::moveToGroundCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->robotSkill.moveToGround();
    return true;
}

bool HighLevelService::scanWorkspaceCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->robotSkill.scanWorkspace();
    return true;
}

bool HighLevelService::pickAndPlaceCallback(franka_ros_example::frankX_pickup::Request& req,
                franka_ros_example::frankX_pickup::Response& res)
{
    Pose pick_position;
    pick_position.x = req.pick_x;
    pick_position.y = req.pick_y;
    pick_position.z = 0;

    Pose place_position;
    place_position.x = req.place_x;
    place_position.y = req.place_y;
    place_position.z = 0;
    
    this->robotSkill.pickAndPlaceSkill(pick_position,place_position);
    
    return true;
}

bool HighLevelService::scanOneBlockToPickAndPlaceCallback(franka_ros_example::frankX_pickup::Request& req,
                franka_ros_example::frankX_pickup::Response& res)
{
    Pose pick_position;
   // ros::ServiceClient getPickPosition = 
        n.serviceClient<franka_ros_example::frankX_move>("/computer_vision_service/get_pick_position");
    Pose place_position;
    place_position.x = req.place_x;
    place_position.y = req.place_y;
    place_position.z = 0;

    this->robotSkill.pickAndPlaceSkill(pick_position,place_position);
    return true;
}



