import sys
import numpy as np

import matplotlib.pyplot as plt

import multicopter_mpc

import pinocchio
import crocoddyl
import example_robot_data

from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR, MULTICOPTER_MPC_OCP_DIR

WITHDISPLAY = 'display' in sys.argv
WITHPLOT = 'plot' in sys.argv

uav = example_robot_data.load('iris')
uav_model = uav.model

# # UAV Params
mc_params = multicopter_mpc.MultiCopterBaseParams()
mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")

# Mission
mission = multicopter_mpc.Mission(uav.nq + uav.nv)
mission.fillWaypoints(MULTICOPTER_MPC_MISSION_DIR + "/hover.yaml")

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
trajectory.solver.th_stop = 1e-5
# trajectory.solve(state_guess, control_guess)
trajectory.solve()


fig, axs = plt.subplots(4, 1)
us = np.vstack(trajectory.controls).T
for idx, ax in enumerate(axs):
    ax.plot(us[idx, :])

plt.show()


state_trajectory = trajectory.states
control_trajectory = trajectory.controls
time_lst = [trajectory.dt for i in range(0, trajectory.n_knots + 1)]

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    for idx_wp, wp in enumerate(mission.waypoints):
        name = 'world/wp' + str(idx_wp)
        uav.viewer.gui.addXYZaxis(name, [1., 0., 0., 1.], .03, 0.5)
        wp_pose = pinocchio.SE3ToXYZQUATtuple(wp.pose)
        uav.viewer.gui.applyConfiguration(name, wp_pose)

    display.display(state_trajectory, [], [], time_lst, 1)