#pragma once
#include "robot_controllers/controller/controller.h"
#include "robot_controllers/PickAndPlaceConfig.h"
#include "red_msgs/WorldPoint.h"

struct PickAndPlaceParams
{
    double x_pick;
    double y_pick;
    double z_pick;

    double x_place;
    double y_place;
    double z_place;

    double delta_height;
};

/**
 * controller that allows the user to pick and place objects
*/
class PickAndPlaceController : public Controller
{

public:

    PickAndPlaceController( ros::NodeHandle& n_pick, ros::NodeHandle& n_controller);
    ~PickAndPlaceController();
    /**
     * function used to setup dynamic reconfiguration server
    */
    void activateDynamicServer();

    /**
     * location read from attribute pickAndPlaceParams
     * @return width of object picked, around 0 if no object is picked
    */
    double pick();
    /**
     * location read from attribute pickAndPlaceParams
     * @return true if object is correctly placed
    */
    bool place();
    /**
     * location read from attribute pickAndPlaceParams
     * if an object is picked, then it placces it, otherwise it returns home
     * @return true if an object is correctly placed, false otherwise
    */
    bool pickAndPlace();
    /**
     * location read from camera
     * if an object is picked, then it placces it, otherwise it returns home
     * @return true if an object is correctly placed, false otherwise
    */
    bool scanAndPickAndPlace();

protected:

    /**
     * values of this data structure are updated from the reconfiguration GUI
    */
    PickAndPlaceParams pickAndPlaceParams;

    ros::NodeHandle& n_pick;

    ros::ServiceClient brickDetectionService;
    
    //functions and attributes for the dynamic reconfiguration server
    void reconfigure_pick_and_place_params(
        robot_controllers::PickAndPlaceConfig &config, uint32_t level);
    dynamic_reconfigure::Server<robot_controllers::PickAndPlaceConfig>* 
        pick_and_place_param_server;
    dynamic_reconfigure::Server<robot_controllers::PickAndPlaceConfig>::CallbackType 
        pick_and_place_param_callback;
   
};


