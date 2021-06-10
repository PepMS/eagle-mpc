import sys

import crocoddyl
import pinocchio
import example_robot_data
import numpy as np

import eagle_mpc
import yaml_parser

uav = example_robot_data.loadIris()
link_name = "iris__base_link"
yaml_uav = yaml_parser.ParserYAML("/home/pepms/robotics/libraries/eagle_mpc/config/multirotor/iris.yaml", "",
                                  True)

uav_model = uav.model
base_link = uav_model.getFrameId(link_name)

# UAV Params
server_uav = yaml_parser.ParamsServer(yaml_uav.getParams())
mc_params = eagle_mpc.MultiCopterBaseParams()
mc_params.fill(server_uav)

# Mission
yaml_mission = yaml_parser.ParserYAML("/home/pepms/robotics/libraries/eagle_mpc/config/mission/simple.yaml", "",
                                      True)
server_mission = yaml_parser.ParamsServer(yaml_mission.getParams())
mission = eagle_mpc.Mission(uav.nq + uav.nv)

mission.fillWaypoints(server_mission)
mission.fillInitialState(server_mission)

print("Hello")