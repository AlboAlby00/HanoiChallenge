#pragma once
#include "robot_controllers/controller/controller.h"
#include "std_srvs/Empty.h"


typedef std_srvs::Empty::Request EmptyRequest;
typedef std_srvs::Empty::Response EmptyResponse;

/**
 * class used to advertise all function of controller as service
*/
class ControllerService
{
    public:
        ControllerService(ros::NodeHandle& n, Controller& controller);

    private:

        ros::NodeHandle& n;
        Controller& controller;

        ros::ServiceServer moveHomeService;
        bool moveHomeCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer moveGripperService;
        bool moveGripperCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer clampGripperService;
        bool clampGripperCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer releaseGripperService;
        bool releaseGripperCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer moveCartesianService;
        bool moveCartesianCallback(EmptyRequest& req, EmptyResponse& res);
        
};
