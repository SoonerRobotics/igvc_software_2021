#!/bin/bash

rosparam set port 9090
roslaunch rosbridge_server rosbridge_websocket.launch
