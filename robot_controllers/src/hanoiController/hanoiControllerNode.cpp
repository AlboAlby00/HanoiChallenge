#include "robot_controllers/hanoiController/hanoiControllerService.h"
#include "robot_controllers/pickAndPlaceController/pickAndPlaceControllerService.h"
#include "robot_controllers/controller/controllerService.h"


int main(int argc, char**argv)
{
    ros::init(argc,argv,"hanoi_controller");
    
    // multiple nodes needed since it is possible to advertise only one dynamic
    // reconfiguration server per node, but 3 are required for hanoiController class
    ros::NodeHandle n_hanoi("~");
    ros::NodeHandle n_pick("pick_and_place");
    ros::NodeHandle n_controller("controller");

    HanoiController controller(n_hanoi,n_pick,n_controller);

    // advertise controller functionalities as services
    ControllerService service(n_controller,controller);
    PickAndPlaceControllerService pickAndPlaceService(n_pick,controller);
    HanoiControllerService HanoiControllerService(n_hanoi,controller);

    
    ROS_INFO("hanoi controller spinning");
    ros::spin();

    return 0;
}