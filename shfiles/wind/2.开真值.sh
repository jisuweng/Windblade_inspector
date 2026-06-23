#!/bin/bash
gnome-terminal --window\
    --title="连接mavros" --command "bash -c 'python3 ~/XTDrone/communication/multirotor_communication.py iris 0; exec bash'" \
    --tab --title="开真值" --command "bash -c 'sleep 2; python ~/XTDrone/sensing/pose_ground_truth/get_local_pose.py iris 1 ; exec bash'" \
   
    
    
   
    
    
    






