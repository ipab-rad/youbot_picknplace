#!/bin/sh
# @Author: alex
# @Date:   2015-11-10
# @Last Modified by:   alex
# @Last Modified time: 2015-11-10

ssh youbot@starscream "pkill -xf '/usr/bin/python /opt/ros/hydro/bin/roslaunch youbot_picknplace tabletop.launch'"
