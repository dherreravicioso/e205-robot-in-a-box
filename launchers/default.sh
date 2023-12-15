#!/bin/bash

source /environment.sh

# initialize launch file
dt-launchfile-init

# set module's health
dt-set-module-healthy

# launching app
dt-exec roslaunch e205_top all_drivers.launch 

# wait for app to end
dt-launchfile-join