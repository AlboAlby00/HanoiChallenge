#include "robot_controllers/hanoiController/hanoiControllerService.h"   

HanoiControllerService::HanoiControllerService(ros::NodeHandle& n, HanoiController& c) :
    n(n), controller(c)
{
    this->solveHanoiService = 
        this->n.advertiseService("solve_hanoi", 
            &HanoiControllerService::solveHanoiCallback, this);

    this->measureDiskService =
        this->n.advertiseService("measure_disk",
            &HanoiControllerService::measureDiskCallback, this);

    this->getStateService =
        this->n.advertiseService("get_state",
            &HanoiControllerService::getStateCallback, this);
}

bool HanoiControllerService::solveHanoiCallback(
        EmptyRequest& req, EmptyResponse& res)
{
    bool result = this->controller.solveHanoi();
    return result;
}


bool HanoiControllerService::measureDiskCallback(
    EmptyRequest& req, EmptyResponse& res)
{
    this->controller.measureWidthDisk();
    return true;
}

bool HanoiControllerService::getStateCallback(
    EmptyRequest& req, EmptyResponse& res)
{
    this->controller.getState();
    return true;
}