
#include "robot_controllers/pickAndPlaceController/pickAndPlaceControllerService.h"
#include "robot_controllers/controller/controllerService.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"pick_and_place_controller");
    
    // multiple nodes needed since it is possible to advertise only one dynamic
    // reconfiguration server per node, but 2 are required for pickAndPlaceController class
    ros::NodeHandle n_pick("pick_and_place");
    ros::NodeHandle n_controller("controller");

    PickAndPlaceController controller(n_pick,n_controller);

    ControllerService service(n_controller,controller);
    PickAndPlaceControllerService pickAndPlaceService(n_pick,controller);

    ROS_INFO("pick and place controller spinning");
    ros::spin();

    return 0;
}
