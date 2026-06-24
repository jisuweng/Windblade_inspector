#!/bin/bash
gnome-terminal --window \
    --title="px4ctrl" --command "bash -c ' source ~/ego_ws/devel/setup.bash;roslaunch px4ctrl singl_run.launch; exec bash'" 



    
    







