#include "red_robot_utils/robotSkill.h"

void testPickAndPlace(ros::NodeHandle& nh);
void testMoveToPosition(ros::NodeHandle& nh);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "my_robot_control");
  ros::NodeHandle n("~");

  RealRobotSkill realSkill(n);

  testPickAndPlace(n);

  return 0;
}

void testPickAndPlace(ros::NodeHandle& nh)
{

    RealRobotSkill skill(nh);

    Pose pick_pose;
    pick_pose.x = 0.4025;
    pick_pose.y = -0.245;
    pick_pose.z = 0;

    Pose place_pose;
    place_pose.x = 0.4025;
    place_pose.y = 0.1;
    place_pose.z = 0;

    skill.pickAndPlaceSkill(pick_pose,place_pose);
    skill.moveHome();
}

void testMoveToPosition(ros::NodeHandle& nh)
{
    RealRobotSkill realSkill(nh);
    SimulatedRobotSkill simSkill(nh);

    franka_ros_example::frankX_move ser;

    ser.request.x = 0.2678;
    ser.request.y = -0.215704;
    ser.request.z = 0.84;
    ser.request.a = M_PI_2;
    ser.request.b = 0;
    ser.request.c = 0;
    ser.request.force_limit = 20;
    ser.request.speed = 0.2;
    ser.request.elbow = 0;

    realSkill.move_client.call(ser);
    simSkill.move_client.call(ser);
}