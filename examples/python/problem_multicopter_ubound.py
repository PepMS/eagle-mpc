import os
import sys

import crocoddyl
import pinocchio
import example_robot_data
import numpy as np

import multicopter_mpc
from multicopter_mpc.utils.solver_squash import SolverSquashFDDP
import yaml_parser

WITHPLOT = 'plot' in sys.argv
HECTOR = 'hector' in sys.argv
WITHDISPLAY = 'display' in sys.argv


crocoddyl.switchToNumpyMatrix()

if HECTOR:
    uav = example_robot_data.loadHector()
    link_name = "base_link"
    yaml_uav = yaml_parser.ParserYAML("../../config/multirotor/hector.yaml", "", True)
else:
    uav = example_robot_data.loadIris()
    link_name = "iris__base_link"
    yaml_uav = yaml_parser.ParserYAML("/home/pepms/robotics/libraries/multicopter_mpc/config/multirotor/iris.yaml", "",
                                      True)

uav_model = uav.model
base_link = uav_model.getFrameId(link_name)

# UAV Params
server_uav = yaml_parser.ParamsServer(yaml_uav.getParams())
mc_params = multicopter_mpc.MultiCopterBaseParams()
mc_params.fill(server_uav)

# Mission
yaml_mission = yaml_parser.ParserYAML("/home/pepms/robotics/libraries/multicopter_mpc/config/mission/passthrough.yaml",
                                      "", True)
server_mission = yaml_parser.ParamsServer(yaml_mission.getParams())
mission = multicopter_mpc.Mission(uav.nq + uav.nv)
mission.fillWaypoints(server_mission)
mission.fillInitialState(server_mission)

dt = 1e-2
s_lb = pinocchio.utils.zero([mc_params.n_rotors, 1])
s_ub = pinocchio.utils.zero([mc_params.n_rotors, 1])
s_lb.fill(mc_params.min_thrust)
s_ub.fill(mc_params.max_thrust)

state = crocoddyl.StateMultibody(uav_model)
actuation = crocoddyl.ActuationModelMultiCopterBase(state, mc_params.n_rotors, mc_params.tau_f)
squashing = crocoddyl.SquashingModelSmoothSat(s_lb, s_ub, mc_params.n_rotors)
mc_actuation = crocoddyl.ActuationSquashingModel(actuation, squashing, actuation.nu)
problem_mission = multicopter_mpc.ProblemMission(mission, mc_params, uav_model, mc_actuation,
                                                 uav_model.getFrameId(link_name), dt)
problem = problem_mission.createProblem()
solver = SolverSquashFDDP(problem, squashing)
solver.setCallbacks([crocoddyl.CallbackLogger(), crocoddyl.CallbackVerbose()])
solver.solve()

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    uav.viewer.gui.addXYZaxis('world/wp', [1., 0., 0., 1.], .03, 0.5)
    # hector.viewer.gui.applyConfiguration(
    #     'world/wp',
    #     target_pos.tolist() + [target_quat[0], target_quat[1], target_quat[2], target_quat[3]])

    display.displayFromSolver(solver)

if WITHPLOT:
    log = solver.getCallbacks()[0]
    crocoddyl.plotOCSolution(log.xs, log.us, figIndex=1, show=False)
    crocoddyl.plotConvergence(log.costs, log.u_regs, log.x_regs, log.stops, log.grads, log.steps, figIndex=2)