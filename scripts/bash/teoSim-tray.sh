#!/bin/bash

# Dependencies:
# - https://github.com/roboticslab-uc3m/teo-openrave-models  (provides teo.robot.xml)
# - https://github.com/roboticslab-uc3m/openrave-yarp-plugins  (provides OpenraveYarpPluginLoader)

openrave ./teo/teo.robot.xml --module OpenraveYarpPluginLoader "open --device controlBoard_nws_yarp --subdevice YarpOpenraveControlboard --robotIndex 0 --allManipulators"
