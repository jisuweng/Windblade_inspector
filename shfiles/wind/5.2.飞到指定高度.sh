#!/bin/bash
gnome-terminal --window \
    --title="px4ctrl飞到指定高度悬停" --command "bash -c 'source /opt/ros/noetic/setup.bash; source ~/ego_ws/devel/setup.bash; source ~/WTBinspector_ws/devel/setup.bash 2>/dev/null || true; cd ~/WTBinspector_ws; python3 scripts/px4ctrl_climb_forever.py --speed 2.0; exec bash'"
