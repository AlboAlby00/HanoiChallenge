#include "robot_controllers/controller/controllerService.h"


int main(int argc, char**argv)
{
    ros::init(argc,argv,"red_controller");
    ros::NodeHandle n("~");

    Controller controller(n);
    // advertise controller functionalities as services
    ControllerService service(n,controller);

    ROS_INFO("controller spinning");
    ros::spin();

    return 0;
}