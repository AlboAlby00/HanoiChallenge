#include "robot_controllers/pickAndPlaceController/pickAndPlaceControllerService.h"

PickAndPlaceControllerService::PickAndPlaceControllerService(ros::NodeHandle& n, PickAndPlaceController& c) : 
    controller(c),n(n)
{
    this->pickService = 
        this->n.advertiseService("pick", 
            &PickAndPlaceControllerService::pickCallback, this);

    this->placeServer = 
        this->n.advertiseService("place",
            &PickAndPlaceControllerService::placeCallback, this);

    this->pickAndPlaceServer = 
        this->n.advertiseService("pickAndPlace",
            &PickAndPlaceControllerService::pickAndPlaceCallback, this);

    this->scanAndPickAndPlaceServer = 
        this->n.advertiseService("scanAndPickAndPlace",
            &PickAndPlaceControllerService::scanAndPickAndPlaceCallback, this);
}

bool PickAndPlaceControllerService::pickCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.pick();
    return true;
}

bool PickAndPlaceControllerService::placeCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.place();
    return true;
}

bool PickAndPlaceControllerService::pickAndPlaceCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.pickAndPlace();
    return true;
}

bool PickAndPlaceControllerService::scanAndPickAndPlaceCallback(EmptyRequest& req, EmptyResponse& res)
{
    this->controller.scanAndPickAndPlace();
    return true;
}