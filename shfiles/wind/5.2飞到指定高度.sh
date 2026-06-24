#!/bin/bash
gnome-terminal --window \
    --title="px4ctrl飞到80m悬停" --command "bash -c 'source /opt/ros/noetic/setup.bash; source ~/ego_ws/devel/setup.bash; cd ~/WTBinspector_ws; python3 scripts/px4ctrl_climb_forever.py --speed 2.0 --target-height 83.0; exec bash'"
