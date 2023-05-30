#!/usr/bin/env python3
import rospy
from red_msgs.srv import *

class HanoiLogic:
    """
            Class in which it is implemented the hanoi logic
            Communicates to HanoiState through the service client "get_state" and
            advertise a service giving the optimal action the robot should perform according to the state
    """

    def __init__(self) -> None:

        print("launching hanoi logic node")

        self.get_command = rospy.Service('hanoi_logic/get_command', Command , self.callback_get_command)

        rospy.wait_for_service('hanoi_state/get_state')
        try:
            self.get_state = rospy.ServiceProxy('hanoi_state/get_state', State)   
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        print("hanoi node correctly launched")


    def callback_get_command(self,req)->None:

        res = self.get_state(StateRequest())
        state = list(res.diskPositions)
       
        self.plan = []
        print("plan: ")
        
        self.moveDisks(state,largestToMove=0,targetPosition=0)

        # checking if the state is a final position
        if self.plan:
            command = self.plan[0]
            # find number of disks below the disk to move
            disk_start_position = command[0]
            res = self.get_state(StateRequest())
            state = list(res.diskPositions)
            n_disks_below = state.count(disk_start_position) - 1
            command.append(n_disks_below)
            # is final position set to false  
            command.append(False)
            return CommandResponse(*command) #using unpacking

        else: 
            # is final position set to true
            return CommandResponse(None,None,None,None,True)


    # https://stackoverflow.com/questions/49220476/tower-of-hanoi-solving-halfway-algorithm-in-python
    
    def moveDisks(self, diskPositions : list, largestToMove :int, targetPosition : int) -> None:
        """
            recursive function used to calculate the optimal plan to solve hanoi
            given the current diskPositions
        """
        for disk in range(largestToMove, len(diskPositions)):
            
            currentPosition = diskPositions[disk]         
            
            if currentPosition != targetPosition:
                
                otherPosition = 3 - targetPosition - currentPosition
                self.moveDisks(diskPositions, disk+1, otherPosition)
                
                print ("Move ", disk, " from ", currentPosition, " to ", targetPosition)
                
                self.plan.append([currentPosition,targetPosition,disk])
                diskPositions[disk]=targetPosition
                self.moveDisks(diskPositions, disk+1, targetPosition)
                
                break


# launching the node
if __name__ == '__main__':
    rospy.init_node('hanoi_logic')
    try:
        HanoiLogic()
    except rospy.ROSInterruptException:
        print("Error launching hanoi logic") 
    rospy.spin()