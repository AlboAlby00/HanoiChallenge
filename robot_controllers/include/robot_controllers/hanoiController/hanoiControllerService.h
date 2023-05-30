#pragma once
#include "robot_controllers/hanoiController/hanoiController.h"
#include "std_srvs/Empty.h"
#include "red_msgs/Command.h"
#include "red_msgs/DiskMovement.h"


typedef std_srvs::Empty::Request EmptyRequest;
typedef std_srvs::Empty::Response EmptyResponse;

/**
 * class used to advertise all the functionality of hanoiController as service
*/
class HanoiControllerService
{
    public:
        HanoiControllerService(ros::NodeHandle& n, HanoiController& c);

    private:

        ros::NodeHandle& n;
        HanoiController& controller;

        ros::ServiceServer solveHanoiService;
        bool solveHanoiCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer measureDiskService;
        bool measureDiskCallback(
            EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer getStateService;
        bool getStateCallback(
            EmptyRequest& req, EmptyResponse& res);
        
        
};