#!/usr/bin/env python3

from red_msgs.srv import *

import rospy

class HanoiState:

    """
        Class that abstract the current state of the hanoi game
        advertise two services,
        "update_state" to update the state after a disk movement,
        "get_state" to share the current state with other nodess

    """

    def __init__(self) -> None:

        print("launching hanoi state node")

        # notation is, from biggest disk to smallest disk, 
        # [disk0 position, disk1 position, ...]
        self.diskPosition = [2,2,2]
        
        self.get_state_server = rospy.Service(
            '/hanoi_state/get_state', State, self.callback_send_state)
        self.update_state_server = rospy.Service(
            '/hanoi_state/update_state', DiskMovement, self.callback_update_state)
        self.set_state_server = rospy.Service(
            '/hanoi_state/set_state', DisksWidth, self.callback_set_state)

        print("hanoi test node correctly launched")

    def callback_send_state(self,req):
        res = StateResponse(self.diskPosition)
        return res
    
    # service called by hanoi controller after moving a disk
    def callback_update_state(self,req):
        # get index of disk to move
        diskIndex = len(self.diskPosition) - self.diskPosition[::-1].index(req.source) - 1
        print("disk index moved: "+str(diskIndex))
        # move disk
        self.diskPosition[diskIndex] = req.dest
        print("State updated: "+str(self.diskPosition))
        return DiskMovementResponse(True)
    
    # convert req in  the self.diskPosition format
    def callback_set_state(self,req):
        
        disks = []
        countIndex = 0

        print("disk width: "+str(req.disksWidth))
        

        for rodIndex,numberDisks in enumerate(req.numberDisks):
            diskIndex=0
            while diskIndex<numberDisks:
                print("count index: "+str(countIndex))
                diskWidth = req.disksWidth[countIndex]
                disk = {
                    "width": diskWidth,
                    "rod": rodIndex
                }
                countIndex += 1
                diskIndex += 1
                disks.append(disk)

        # ordering disks in decreasing order
        sortedDisks = sorted(disks, key=lambda x: x['width'],reverse=True)

        print(disks)
        print(sortedDisks)

        self.diskPosition = []
        for disk in sortedDisks:
            self.diskPosition.append(disk["rod"])

        print("State set: "+str(self.diskPosition))

        return DisksWidthResponse()


        

if __name__ == "__main__":
    rospy.init_node("hanoi_test_node")
    HanoiState()
    rospy.spin()