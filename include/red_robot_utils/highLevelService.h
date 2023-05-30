#include "my_robot_control/robotSkill.h"
#include "std_srvs/Empty.h"
#include "franka_ros_example/frankX_end_effector_T.h"



typedef std_srvs::Empty::Request EmptyRequest;
typedef std_srvs::Empty::Response EmptyResponse;


class HighLevelService
{
    public:
        HighLevelService(ros::NodeHandle& n, RobotSkill robotSkill);

    private:

        ros::NodeHandle n;
        RobotSkill robotSkill;

        ros::ServiceServer moveHomeService;
        ros::ServiceServer moveToGroundService;
        ros::ServiceServer scanWorkspaceService;
        ros::ServiceServer pickAndPlaceService;
        ros::ServiceServer scanOneObjectToPickAndPlaceService;

        bool moveHomeCallback(EmptyRequest& req, EmptyResponse& res);
        bool moveToGroundCallback(EmptyRequest& req, EmptyResponse& res);
        bool scanWorkspaceCallback(EmptyRequest& req, EmptyResponse& res);
        bool pickAndPlaceCallback(franka_ros_example::frankX_pickup::Request& req,
                franka_ros_example::frankX_pickup::Response& res);
        bool scanOneBlockToPickAndPlaceCallback(franka_ros_example::frankX_pickup::Request& req,
                franka_ros_example::frankX_pickup::Response& res);


};