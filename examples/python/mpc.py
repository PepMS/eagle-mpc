import numpy as np
import time

import crocoddyl

import multicopter_mpc
from multicopter_mpc.utils.simulator import AerialSimulator

# Trajectory
dt = 10  # ms
useSquash = True
robotName = 'hexacopter370'
trajectoryName = 'hover'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/" + robotName + '_' +
                     trajectoryName + ".yaml")

problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")
if useSquash:
    solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
else:
    solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

mpcController = multicopter_mpc.CarrotMpc(
    trajectory, solver.xs, dt, "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc/" + robotName + "_mpc.yaml")
mpcController.updateProblem(0)
mpcController.solver.solve(solver.xs[:mpcController.problem.T + 1], solver.us[:mpcController.problem.T])

simulator = AerialSimulator(mpcController.robot_model, mpcController.platform_params, mpcController.dt, solver.xs[0])
t = 0
updateTime = []
solveTime = []
for i in range(0, problem.T * 2):
    mpcController.problem.x0 = simulator.states[-1]
    start = time.time()
    mpcController.updateProblem(t)
    end = time.time()
    updateTime.append(end - start)
    start = time.time()
    mpcController.solver.solve(mpcController.solver.xs, mpcController.solver.us, mpcController.iters)
    print(mpcController.solver.us_squash[21])
    end = time.time()
    solveTime.append(end - start)
    control = np.copy(mpcController.solver.us_squash[0])
    simulator.simulateStep(control)
    t += mpcController.dt

print("Average update time: ", sum(updateTime) / len(updateTime))
print("Average solving time: ", sum(solveTime) / len(solveTime))
