#include "red_robot_utils/highLevelService.h"


int main(int argc, char** argv)
{
    ros::init(argc,argv,"high_level_service_sim");

    ros::NodeHandle n("~");

    double timeInHz = 1000;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate loop_rate(timeInHz);

    SimulatedRobotSkill simulatedRobotSkill(n);
    HighLevelService highLevelService(n,simulatedRobotSkill);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}