import os
import sys

import crocoddyl
import pinocchio
import numpy as np
import example_robot_data

import multicopter_mpc
import yaml_parser

WITHDISPLAY = 'display' in sys.argv or 'DISPLAY' in os.environ
WITHPLOT = 'plot' in sys.argv or 'PLOT' in os.environ
HECTOR = 'hector' in sys.argv

crocoddyl.switchToNumpyMatrix()

if HECTOR:
  uav = example_robot_data.loadHector()
  link_name = "base_link"
  yaml_uav = yaml_parser.ParserYAML("../../config/multirotor/hector.yaml", "", True)
else:
  uav = example_robot_data.loadIris()
  link_name = "iris__base_link"
  yaml_uav = yaml_parser.ParserYAML("../../config/multirotor/iris.yaml", "", True)

uav_model = uav.model
base_link = uav_model.getFrameId(link_name)

# UAV Params
server_uav = yaml_parser.ParamsServer(yaml_uav.getParams())
mc_params = multicopter_mpc.MultiCopterBaseParams()

# Mission
# yaml_mission = yaml_parser.ParserYAML("/home/pepms/robotics/libraries/optiuavm/mission/loop.yaml", "", True)
# server_mission = yaml_parser.ParamsServer(yaml_mission.getParams())


# wp_list = optiuavm.fillWaypoints(server_mission)
# x0 = optiuavm.fillInitialState(server_mission)