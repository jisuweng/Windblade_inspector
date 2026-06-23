#!/bin/bash
rm ~/.ros/eeprom/parameters*
rm -rf ~/.ros/sitl*
killall -9 gzserver
killall -9 gzclient
killall -9 rosmaster
killall -9 rosnode
