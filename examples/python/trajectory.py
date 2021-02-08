import sys
import numpy as np
import matplotlib.pyplot as plt

import crocoddyl
import multicopter_mpc
import example_robot_data

WITHDISPLAY = 'display' in sys.argv

trajectory = multicopter_mpc.Trajectory()

parserYaml = multicopter_mpc.ParserYaml("quad_loop.yaml",
                                        "/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory")
paramsServer = multicopter_mpc.ParamsServer(parserYaml.params())

trajectory.autoSetup(paramsServer)
x0 = trajectory.state.zero()
problem = trajectory.createProblem(10, True, x0, "IntegratedActionModelEuler")
solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)

# problem = trajectory.createProblem(10, False, x0, "IntegratedActionModelEuler")
# solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])

# Solving
if solver.solve([], [], maxiter=400):
    print("Solved successfully!")
else:
    print("Solver exit with error")

fig, axs = plt.subplots(4, 1)
us = np.vstack(solver.us).T
for idx, ax in enumerate(axs):
    ax.plot(us[idx, :])

plt.show()

print (solver.xs[-1])
uav = example_robot_data.load('iris')

if WITHDISPLAY:
    display = crocoddyl.GepettoDisplay(uav)
    display.displayFromSolver(solver)
