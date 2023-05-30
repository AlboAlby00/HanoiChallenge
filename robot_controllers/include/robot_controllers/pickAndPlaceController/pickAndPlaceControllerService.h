#pragma once
#include "robot_controllers/pickAndPlaceController/pickAndPlaceController.h"
#include "std_srvs/Empty.h"

typedef std_srvs::Empty::Request EmptyRequest;
typedef std_srvs::Empty::Response EmptyResponse;

class PickAndPlaceControllerService
{
    public:
        PickAndPlaceControllerService(ros::NodeHandle& n, PickAndPlaceController& c);

    private:

        ros::NodeHandle& n;
        PickAndPlaceController& controller;

        ros::ServiceServer pickService;
        bool pickCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer placeServer;
        bool placeCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer pickAndPlaceServer;
        bool pickAndPlaceCallback(EmptyRequest& req, EmptyResponse& res);

        ros::ServiceServer scanAndPickAndPlaceServer;
        bool scanAndPickAndPlaceCallback(EmptyRequest& req, EmptyResponse& res);

        
        
};
