#include "red_robot_utils/simulatedRobot.h"


SimulatedRobot::SimulatedRobot(ros::Rate loop_rate_, ros::NodeHandle& n) : loop_rate(loop_rate_), n(n)
{
    this->jointPub =
        this->n.advertise<sensor_msgs::JointState>("/joint_position_example_controller/JointPosition", 10);
    
    this->trajectoryInterface.setDOF(7);
    this->trajectoryInterface.setUpdateRate(0.1);

    std::vector<double> max = {0,0,0,0,0,0,0};
    this->trajectoryInterface.setMaxVel(max);
    this->trajectoryInterface.setMaxAcc(max);
    this->trajectoryInterface.setMaxJerk(max);
}

bool SimulatedRobot::service_move_call(moveRequest  &req, moveResponse &res)
{   
  
    moveit::planning_interface::MoveGroupInterface::Plan plan = 
        this->trajectoryInterface.generateCartesianMotion("~", "~", req.x , req.y , req.z);
    sendToRobot(plan);
    return true;
}

bool SimulatedRobot::service_joint_call(jointRequest  &req, jointResponse &res)
{
    Eigen::VectorXd* current_joint_pose;
    current_joint_pose = this->get_current_joint_pose();

    Eigen::VectorXd joint_pose_requested(7);
    joint_pose_requested <<  req.q0 , req.q1 , req.q2 , req.q3 , req.q4 , req.q5 , req.q6;

    std::list<Eigen::VectorXd> waypoints;
    waypoints.push_back(*current_joint_pose);
    waypoints.push_back(joint_pose_requested);
        
    //std::cout<<"current pose: " << waypoints.front() << std::endl;
    //std::cout<<"target pose: " << waypoints.back() << std::endl;

    MotionData trajectory = 
        this->trajectoryInterface.generateJointTrajectory(waypoints);
    sendToRobot(trajectory);

    free(current_joint_pose);

    return true;
}

bool SimulatedRobot::service_gripper_call(gripperRequest& req, gripperResponse& res)
{
    actionlib::SimpleActionClient<franka_gripper::MoveAction> gripper_action("/franka_gripper/move", true);
    gripper_action.waitForServer();

    franka_gripper::MoveGoal openGoal;
    openGoal.speed=0.01;
    openGoal.width = req.option=="release" ? req.release_width : req.move_width;
    gripper_action.sendGoal(openGoal);

    bool finished_before_timeout = gripper_action.waitForResult(ros::Duration(3000.0));
    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = gripper_action.getState();
        ROS_INFO("Gripper opened: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Gripper did not open before the time out.");
        return false;
    }
    return true;
}

bool SimulatedRobot::service_grasp_call(graspRequest& req, graspResponse& res)
{
    actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_action("/franka_gripper/grasp", true);

    franka_gripper::GraspGoal graspGoal;
    graspGoal.epsilon.inner = req.epsilon_inner;
    graspGoal.epsilon.outer = req.epsilon_outer;
    graspGoal.force = req.force;
    graspGoal.speed = req.speed;
    graspGoal.width = req.width;

    grasp_action.waitForServer();
    grasp_action.sendGoal(graspGoal);
    bool finished_before_timeout = grasp_action.waitForResult(ros::Duration(3000.0));


    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = grasp_action.getState();
        ROS_INFO("Gripper closed: %s", state.toString().c_str());
    }
    else
    {
        ROS_INFO("Gripper did not close before the time out.");
        return false;
    }

    auto result = grasp_action.getResult();
    res.object_grasped = result->success;

    return true;
}

Eigen::VectorXd* SimulatedRobot::get_current_joint_pose()
{
    std::string topic_joint = "/joint_states";
    sensor_msgs::JointStateConstPtr my_joint_state = 
        ros::topic::waitForMessage<sensor_msgs::JointState>(topic_joint);
    uint jointnumber=7;
    
    Eigen::VectorXd* current_pose = new Eigen::VectorXd(jointnumber);
    for (size_t i = 0; i < jointnumber; ++i)
    {
        (*current_pose)(i) = my_joint_state->position[i];
    }
    return current_pose;
}

void SimulatedRobot::sendToRobot(const moveit::planning_interface::MoveGroupInterface::Plan &myPlan)
{
    int count = 0;
    while (count < myPlan.trajectory_.joint_trajectory.points.size())
    {
        sensor_msgs::JointState msg;
        msg.position.resize(7);
        for (std::size_t i = 0; i < 7; ++i)
        {
            msg.position[i] = myPlan.trajectory_.joint_trajectory.points[count].positions[i];
            //std::cout<<msg.position[i]<<" ";
        }
        // std::cout<<std::endl;
        this->jointPub.publish(msg);

        ros::spinOnce();

        this->loop_rate.sleep();

        ++count;
    }
}

void SimulatedRobot::sendToRobot(const MotionData& data)
{
    sensor_msgs::JointState msg;
    msg.position.resize(7);
   
    for (auto const& point : data.points) 
    {
        for (std::size_t i = 0; i < 7; ++i)
        {
            msg.position[i] = point[i];
        }
    }

    this->jointPub.publish(msg);

    ros::spinOnce();

    this->loop_rate.sleep();

}