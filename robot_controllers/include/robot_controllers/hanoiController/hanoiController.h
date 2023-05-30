#pragma once
#include "robot_controllers/pickAndPlaceController/pickAndPlaceController.h"
#include "robot_controllers/HanoiConfig.h"
#include "red_msgs/DiskMovement.h"
#include "red_msgs/Command.h"
#include "red_msgs/DisksWidth.h"
#include "red_msgs/RodsPosition.h"
#include "std_srvs/Empty.h"
#include <vector>

struct HanoiParams
{

    Pose pose_2 = {0.479 , 0.0, 0.086};
    Pose pose_1 = {0.4025 , 0.0, 0.086};
    Pose pose_0 = {0.33 , 0.0, 0.086};

    Pose poses[3] = {pose_0,pose_1,pose_2};

    double z_base = 0.088;

    double height_drop = 0.19;

    double disk_height = 0.008;
    // if the object grasped is smaller than this, it is recognised as a rod
    double rods_width_threshold = 0.012;

    // 7.6 cm distance between positions
    std::vector<double> disks_width;

    // if true, rods position is get using the camera, otherwise it uses the hardcoded poses
    bool using_computer_vision = false;
};

//enum used to identify the three rods possible for a disk in Hanoi 
enum rod { rod_0, rod_1, rod_2};

/**
 * controller used to solve Hanoi tower
*/
class HanoiController : public PickAndPlaceController
{
    public:
        HanoiController( ros::NodeHandle& hanoi_n ,ros::NodeHandle& pick_n, ros::NodeHandle& n);
        ~HanoiController();
        /**
         * position location read from attribute "hanoiParams"
         * @param source rod from which the disk is picked
         * @param n_disks_below number of disks below the selected disk to move
         * @return the index of the disk picked, -1 if it is a rod, 
         * 0 is the biggest, and then in decreasing order
        */
        int pickDisk(rod source, int n_disks_below);
        /**
         * position location read from attribute "hanoiParams", action executed only if disk picked is
         * disk_to_pick
         * @param source rod from which the disk is picked
         * @param target disk is moved to this rod
         * @param n_disks_below number of disks below the selected disk to move
         * @param disk_to_pick the index of the disk you want to pick, [0 is the bigger, then decreasing]
         * @return true if disk picked is disk_to_pick, false otherwise
        */
        bool pickAndPlaceDisk(rod source, rod target, int n_disks_below, int disk_to_pick);
        /**
         * reads position from "robotParams" attribute
         * @return width of the disk/rod width, if nothing is
         * grasped, then it returns a value close to 0
        */
        double measureWidthDisk();
        /**
         * moves the robot arm "tasting" the positions to sense the state
         * and then sends it to HanoiState node
        */
        bool getState();
        /**
         * solve Hanoi game, position is hardcoded or obtained from camera based on
         * hanoiParams.using_computer_vision
         * @return true when the game is solved
        */
        bool solveHanoi();
        /**
         * try to pick a disk, if detects no disk, it doesn't pick
         * @return the width of the disk picked
        */
       double pick();




    protected:

        /**
         * updates rods positions in hanoiParams getting the data from camera
        */
        void update_rods_positions();
        /**
        * values of this data structure are updated from the reconfiguration GUI
        */
        HanoiParams hanoiParams;
        /**
         * service used to ask hanoi logic node the next move to perform
        */
        ros::ServiceClient commandClient;
        /**
         * service used to notify the hanoi state node that the robot moved a disk
        */
        ros::ServiceClient updateStateClient;
        /**
         * service used to set the initial state of the hanoi game
        */
        ros::ServiceClient setStateClient;
        /**
         * service used to get rods x coordinates
        */
        ros::ServiceClient getRodsPositionsClient;

        // function and attributes used for the dynamic reconfiguration server
        void reconfigure_hanoi_params(
            robot_controllers::HanoiConfig &config, uint32_t level);
        dynamic_reconfigure::Server<robot_controllers::HanoiConfig>* 
            hanoi_param_server;
        dynamic_reconfigure::Server<robot_controllers::HanoiConfig>::CallbackType 
            hanoi_param_callback;


};