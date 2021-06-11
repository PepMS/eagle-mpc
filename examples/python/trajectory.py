import sys
import matplotlib.pyplot as plt
import numpy as np

import crocoddyl
import eagle_mpc
import example_robot_data

WITHDISPLAY = 'display' in sys.argv

dt = 10  # ms
useSquash = True
robotName = 'hexacopter370'
trajectoryName = 'hover'

trajectory = eagle_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/eagle_mpc/eagle_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")
problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")

if useSquash:
    solver = eagle_mpc.SolverSbFDDP(problem, trajectory.squash)
else:
    solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=2)

fig, axs = plt.subplots(6, 1)
us = np.vstack(solver.us_squash).T
# us = np.vstack(solver.us).T
for idx, ax in enumerate(axs):
    ax.plot(us[idx, :])

plt.show()

if WITHDISPLAY:
    robot = example_robot_data.load(trajectory.robot_model.name)
    display = crocoddyl.GepettoDisplay(robot)
    display.displayFromSolver(solver)
