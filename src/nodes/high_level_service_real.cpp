#include "red_robot_utils/highLevelService.h"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"high_level_service_real");

    ros::NodeHandle n("~");

    double timeInHz = 1000;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Rate loop_rate(timeInHz);

    RealRobotSkill realRobotSkill(n);
    HighLevelService highLevelService(n,realRobotSkill);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}