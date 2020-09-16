import sys
import numpy as np

import multicopter_mpc

import pinocchio
import crocoddyl
import example_robot_data
import yaml_parser

from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR, MULTICOPTER_MPC_OCP_DIR

WITHDISPLAY = 'display' in sys.argv
WITHPLOT = 'plot' in sys.argv

uav = example_robot_data.loadIris()
uav_model = uav.model

# # UAV Params
mc_params = multicopter_mpc.MultiCopterBaseParams()
mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")

# Mission
mission = multicopter_mpc.Mission(uav.nq + uav.nv)
mission.fillWaypoints(MULTICOPTER_MPC_MISSION_DIR + "/simple.yaml")

# dt = 1e-2
trajectory = multicopter_mpc.TrajectoryGenerator(uav_model, mc_params, mission)
trajectory.loadParameters(MULTICOPTER_MPC_OCP_DIR + "/trajectory-generator.yaml")

trajectory.createProblem(multicopter_mpc.SolverType.SolverTypeBoxFDDP,
                         multicopter_mpc.IntegratorType.IntegratorTypeEuler, trajectory.dt)
trajectory.setSolverCallbacks(True)

state_guess = mission.interpolateTrajectory("cold")
control = pinocchio.utils.zero(4)
# control += 3.7
control_guess = [control for _ in range(0, len(state_guess) - 1)]
trajectory.setSolverIters(500)
trajectory.setSolverCallbacks(True)
trajectory.solve(state_guess, control_guess)
# trajectory.solve()

state_trajectory = trajectory.getStateTrajectory(0, trajectory.n_knots - 1)
control_trajectory = trajectory.getControlTrajectory(0, trajectory.n_knots - 2)
time_lst = [trajectory.dt for i in range(0, trajectory.n_knots + 1)]

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    for idx_wp, wp in enumerate(mission.waypoints):
        name = 'world/wp' + str(idx_wp)
        uav.viewer.gui.addXYZaxis(name, [1., 0., 0., 1.], .03, 0.5)
        wp_pose = pinocchio.SE3ToXYZQUATtuple(wp.pose)
        uav.viewer.gui.applyConfiguration(name, wp_pose)

    display.display(state_trajectory, [], [], time_lst, 1)

# crocoddyl.plotOCSolution(state_trajectory, control_trajectory, figIndex=1, show=True)
