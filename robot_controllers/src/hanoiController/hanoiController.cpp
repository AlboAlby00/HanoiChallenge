#include "robot_controllers/hanoiController/hanoiController.h"

HanoiController::HanoiController(ros::NodeHandle& hanoi_n,ros::NodeHandle& pick_n, ros::NodeHandle& n) : 
    PickAndPlaceController(pick_n,n)
{
    // initialize client services
    ros::service::waitForService("/hanoi_logic/get_command", ros::Duration(5.0));
    this->commandClient = 
        n.serviceClient<red_msgs::Command>("/hanoi_logic/get_command");

    ros::service::waitForService("/hanoi_state/update_state", ros::Duration(5.0));
    this->updateStateClient = 
        n.serviceClient<red_msgs::DiskMovement>("/hanoi_state/update_state");

    ros::service::waitForService("/hanoi_state/set_state", ros::Duration(5.0));
    this->setStateClient = 
        n.serviceClient<red_msgs::DisksWidth>("/hanoi_state/set_state");

    ros::service::waitForService("/computer_vision/get_rods_positions", ros::Duration(5.0));
    this->getRodsPositionsClient = 
        n.serviceClient<red_msgs::RodsPosition>("/computer_vision/get_rods_position");

    // initialize dynamic reconfigure server
    this->hanoi_param_server = 
        new dynamic_reconfigure::Server<robot_controllers::HanoiConfig>(hanoi_n);
    this->hanoi_param_callback = 
        boost::bind(&HanoiController::reconfigure_hanoi_params, this, _1, _2);
    this->hanoi_param_server->setCallback(this->hanoi_param_callback);


}

void HanoiController::reconfigure_hanoi_params(
    robot_controllers::HanoiConfig &config, uint32_t level)
{
    this->hanoiParams.poses[0].x = config.x_position_0;
    this->hanoiParams.poses[1].x = config.x_position_1;
    this->hanoiParams.poses[2].x = config.x_position_2;

    this->hanoiParams.disk_height = config.disk_height;
    this->hanoiParams.height_drop = config.height_drop;

    this->hanoiParams.rods_width_threshold = config.rods_width_threshold;

    this->hanoiParams.using_computer_vision = config.using_computer_vision;
    
    ROS_INFO("reconfiguring hanoi params");
}

HanoiController::~HanoiController()
{
    free(hanoi_param_server);
}

double HanoiController::pick()
{
    this->robotParams.gripper_width = 0.08;
    this->moveGripper();

    this->robotParams.x = this->pickAndPlaceParams.x_pick;
    this->robotParams.y = this->pickAndPlaceParams.y_pick;
    this->robotParams.z = this->pickAndPlaceParams.z_pick + this->pickAndPlaceParams.delta_height;
    this->moveCartesian();

    this->robotParams.x = this->pickAndPlaceParams.x_pick;
    this->robotParams.y = this->pickAndPlaceParams.y_pick;
    this->robotParams.z = this->pickAndPlaceParams.z_pick;
    this->moveCartesian();
    
    double width = this->clampGripper();
    if(width < this->hanoiParams.rods_width_threshold )
    {
        this->robotParams.gripper_width = 0.02;
        this->releaseGripper();
        return -1;
    }

    this->robotParams.x = this->pickAndPlaceParams.x_pick;
    this->robotParams.y = this->pickAndPlaceParams.y_pick;
    this->robotParams.z = this->pickAndPlaceParams.z_pick + this->pickAndPlaceParams.delta_height;
    this->moveCartesian();
    
    return width;
}

int HanoiController::pickDisk(rod source, int n_disks_below)
{

    pickAndPlaceParams.x_pick = hanoiParams.poses[source].x;
    pickAndPlaceParams.y_pick = hanoiParams.poses[source].y;
    pickAndPlaceParams.z_pick = hanoiParams.poses[source].z + hanoiParams.disk_height * n_disks_below;

    double width = this->pick();

    // rod detected
    if(width==-1)
    {
        return -1;
    }

    // find which disk was picked comparing the width with hanoiParams.disks_width
    int closestIndex = 0;
    double minDifference = std::abs(this->hanoiParams.disks_width[0] - width);

    for (int i = 1; i < this->hanoiParams.disks_width.size(); i++) {
        double difference = std::abs(this->hanoiParams.disks_width[i] - width);
        if (difference < minDifference) {
            minDifference = difference;
            closestIndex = i;
        }
    }

    return closestIndex;
}

bool HanoiController::pickAndPlaceDisk(rod source, rod target, int n_disks_below, int disk_to_pick)
{
    pickAndPlaceParams.delta_height = 0.15;

    ROS_INFO("moving disk from %d to %d", source, target);

    robotParams.a = 0;
    robotParams.b = 0;
    robotParams.c = 0;

    int disk_picked = this->pickDisk(source,n_disks_below);
    bool picked_correct_disk = (disk_picked == disk_to_pick);
    if( picked_correct_disk)
    {
        // place to target location
        pickAndPlaceParams.x_place = hanoiParams.poses[target].x;
        pickAndPlaceParams.y_place = hanoiParams.poses[target].y;
        pickAndPlaceParams.z_place = hanoiParams.poses[target].z + hanoiParams.height_drop;
        this->place();
    }
    else
    {
        // place back to source location
        pickAndPlaceParams.x_place = hanoiParams.poses[source].x;
        pickAndPlaceParams.y_place = hanoiParams.poses[source].y;
        pickAndPlaceParams.z_place = hanoiParams.poses[source].z + hanoiParams.height_drop;
        this->place();
    }

    return picked_correct_disk;

}

double HanoiController::measureWidthDisk()
{
    // move to disk position
    this->moveCartesian();

    // measure disk
    this->robotParams.gripper_width = 0.0;
    double width = this->clampGripper();

    // release gripper
    this->robotParams.gripper_width = 0.02;
    this->releaseGripper();

    return width;

}

bool HanoiController::getState()
{
    this->hanoiParams.disks_width.clear();

    red_msgs::DisksWidth args;
    // open gripper
    this->robotParams.gripper_width = 0.08;
    this->moveGripper();

    this->hanoiParams.disks_width.clear();

    uint rod_index = 0;
    // loop through all three rods
    for( Pose pose : this->hanoiParams.poses )
    {
        uint number_disks_below_current_disk = 0;
        // move to next rod
        this->robotParams.x = pose.x;
            this->robotParams.y = pose.y;
            this->robotParams.z = pose.z + this->hanoiParams.height_drop;
        this->moveCartesian();

        // loop through all disks of a rod
        while(true)
        { 
            //move to next disk
            this->robotParams.x = pose.x;
            this->robotParams.y = pose.y;
            this->robotParams.z = pose.z + 
                number_disks_below_current_disk * this->hanoiParams.disk_height;
            this->moveCartesian();

            // measure width of next disk or rod
            double width = this->measureWidthDisk();
            
            if(width < this->hanoiParams.rods_width_threshold)
            {
                // detected Rod
                ROS_INFO("detected rod of width %f", width);
                args.request.numberDisks[rod_index] = number_disks_below_current_disk;      
                // open gripper
                this->robotParams.gripper_width = 0.08;
                this->moveGripper();
                break; 
            }    
            else
            {
                ROS_INFO("detected disk of width %f", width);
                // add disk to state
                args.request.disksWidth.push_back(width);
                this->hanoiParams.disks_width.push_back(width);
            }
            number_disks_below_current_disk++; // next ineration checking disk above
        }
        rod_index++;
    }
    // send state to hanoi_state
    this->setStateClient.call(args);

    // sort disks from bigger to smaller
    std::sort(
        this->hanoiParams.disks_width.begin(), this->hanoiParams.disks_width.end(), 
        std::greater<double>());
    return true;
    
}

bool HanoiController::solveHanoi()
{
    // if using computer vision, updates rods positions
    ROS_INFO("is using computer vision: %s", this->hanoiParams.using_computer_vision ? "true" : "false");
    if(this->hanoiParams.using_computer_vision)
    {
        this->update_rods_positions();
        ROS_INFO("updating position, x rod 0 is %f", hanoiParams.pose_0.x);
    }

    this->getState();
    // infintie loop until game is solved
    while(true)
    {
        //ask hanoi logic what is the next move 
        red_msgs::Command commandArgs;
        this->commandClient.call(commandArgs);

        if(commandArgs.response.is_final_position)
        {
            ROS_INFO("Hanoi solved!");
            this->moveHome();
            break;
        }

        //move disk
        bool correct_disk_picked = this->pickAndPlaceDisk(
            rod(commandArgs.response.from),
            rod(commandArgs.response.to),
            commandArgs.response.n_disks_below_disk_to_pick,
            commandArgs.response.disk); 

        if(correct_disk_picked)
        {
            //inform hanoi state of disk movement
            red_msgs::DiskMovement diskMovementArgs;
            diskMovementArgs.request.source = commandArgs.response.from;
            diskMovementArgs.request.dest = commandArgs.response.to;
            this->updateStateClient.call(diskMovementArgs);
        }
        else
        {
            ROS_INFO("found inconsistent state");
            this->getState();
        }   
    
    }
    return true;
}

void HanoiController::update_rods_positions()
{
   red_msgs::RodsPosition positions;
   this->getRodsPositionsClient.call(positions);

   this->hanoiParams.pose_0.x = positions.response.rod_0_position[0];
   this->hanoiParams.pose_0.y = positions.response.rod_0_position[1];
   ROS_INFO("hanoiParams.pose_0.x: %f", positions.response.rod_0_position[0] );
   //this->hanoiParams.pose_0.z = positions.response.rod_0_position[2];

   this->hanoiParams.pose_1.x = positions.response.rod_1_position[0];
   this->hanoiParams.pose_1.y = positions.response.rod_1_position[1];
   //this->hanoiParams.pose_1.z = positions.response.rod_1_position[2];

   this->hanoiParams.pose_2.x = positions.response.rod_2_position[0];
   this->hanoiParams.pose_2.y = positions.response.rod_2_position[1];
   //this->hanoiParams.pose_2.z = positions.response.rod_2_position[2];
}

