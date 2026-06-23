#!/bin/bash

gnome-terminal  --window --title="开fastlio" --command "bash -c 'source ~/WTBinspector_ws/devel/setup.bash; 
      roslaunch wtb_pointcloud_mapping wtb_mapping.launch; exec bash'" 
    
   

