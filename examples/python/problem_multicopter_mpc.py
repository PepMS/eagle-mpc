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

# uav = example_robot_data.loadHector()
# link_name = "base_link"
# yaml_uav = yaml_parser.ParserYAML("../../config/multirotor/hector.yaml",
#                                   "", True)

if HECTOR:
    uav = example_robot_data.loadHector()
    link_name = "base_link"
    yaml_uav = yaml_parser.ParserYAML("../../config/multirotor/hector.yaml",
                                      "", True)
else:
    uav = example_robot_data.loadIris()
    link_name = "iris__base_link"
    yaml_uav = yaml_parser.ParserYAML("../../config/multirotor/iris.yaml", "",
                                      True)

uav_model = uav.model
base_link = uav_model.getFrameId(link_name)

# UAV Params
server_uav = yaml_parser.ParamsServer(yaml_uav.getParams())
mc_params = multicopter_mpc.MultiCopterBaseParams()
mc_params.fill(server_uav)

# Mission
yaml_mission = yaml_parser.ParserYAML(
    "/home/pepms/robotics/libraries/optiuavm/mission/simple.yaml", "",
    True)
server_mission = yaml_parser.ParamsServer(yaml_mission.getParams())
mission = multicopter_mpc.Mission(uav.nq + uav.nv)
mission.fillWaypoints(server_mission)
mission.fillInitialState(server_mission)

dt = 1e-2
problem_mission = multicopter_mpc.ProblemMission(
    mission, mc_params, uav_model, uav_model.getFrameId(link_name), dt)
problem = problem_mission.createProblem()

ddp_solver = crocoddyl.SolverFDDP(problem)
ddp_solver.setCallbacks(
    [crocoddyl.CallbackLogger(),
     crocoddyl.CallbackVerbose()])
ddp_solver.solve()

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    uav.viewer.gui.addXYZaxis('world/wp', [1., 0., 0., 1.], .03, 0.5)
    # hector.viewer.gui.applyConfiguration(
    #     'world/wp',
    #     target_pos.tolist() + [target_quat[0], target_quat[1], target_quat[2], target_quat[3]])

    display.displayFromSolver(ddp_solver)
