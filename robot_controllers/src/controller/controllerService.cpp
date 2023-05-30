#include "robot_controllers/controller/controllerService.h"

ControllerService::ControllerService(ros::NodeHandle& n, Controller& c) : 
    controller(c),n(n)
{
    this->moveHomeService = 
        this->n.advertiseService("move_home", 
            &ControllerService::moveHomeCallback, this);

    this->moveGripperService =
        this->n.advertiseService("move_gripper",
            &ControllerService::moveGripperCallback,this);

    this->clampGripperService =
        this->n.advertiseService("clamp_gripper",
            &ControllerService::moveGripperCallback,this);

    this->releaseGripperService =
        this->n.advertiseService("release_gripper",
            &ControllerService::moveGripperCallback,this);

    this->moveCartesianService = 
        this->n.advertiseService("move_cartesian",
            &ControllerService::moveCartesianCallback,this);
}

bool ControllerService::moveHomeCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.moveHome();
    return true;
}

bool ControllerService::moveGripperCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.moveGripper();
    return true;
}

bool ControllerService::clampGripperCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.clampGripper();
    return true;
}

bool ControllerService::releaseGripperCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.releaseGripper();
    return true;
}

bool ControllerService::moveCartesianCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.moveCartesian();
    return true;
}