#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# launch subscriber
roslaunch vectornav vn100_imu_test.launch

# wait for app to end
dt-launchfile-join