#!/bin/bash
gnome-terminal --window --title="风机停机" --command "bash -c 'source ~/WTBinspector_ws/devel/setup.bash; rosrun fans_move stop_fans.py; exec bash'"
