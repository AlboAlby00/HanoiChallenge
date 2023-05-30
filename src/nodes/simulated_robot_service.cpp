#include "red_robot_utils/simulatedRobot.h"
#include "red_robot_utils/robotSkill.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulated_robot_service_node");
    ros::NodeHandle nh("~");

    ros::AsyncSpinner spinner(4);
    spinner.start();

    double timeInHz = 1000;
    ros::Rate loop_rate(timeInHz);
    
    SimulatedRobot simulatedRobot(loop_rate,nh);

    ros::ServiceServer service1 = 
        nh.advertiseService("/simulated_robot_node/frankX_move", &SimulatedRobot::service_move_call, &simulatedRobot);
    ros::ServiceServer service2 = 
        nh.advertiseService("/simulated_robot_node/frankX_gripper", &SimulatedRobot::service_gripper_call, &simulatedRobot);
    ros::ServiceServer service3 = 
        nh.advertiseService("/simulated_robot_node/frankX_joint", &SimulatedRobot::service_joint_call, &simulatedRobot);
    ros::ServiceServer service4 = 
        nh.advertiseService("/simulated_robot_node/frankX_grasp", &SimulatedRobot::service_grasp_call, &simulatedRobot);
        

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    
    return 0;
}
