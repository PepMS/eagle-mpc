import numpy as np
import time

import crocoddyl

import multicopter_mpc
from mpc_controllers import CarrotMpc
from simulator import AerialSimulator

# Trajectory
dt = 10
trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/am_hover.yaml")

problem = trajectory.createProblem(dt, True, "IntegratedActionModelEuler")

solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

mpcController = multicopter_mpc.CarrotMpc(trajectory, solver.xs, dt,
                                          "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc/mpc.yaml")
mpcController.updateProblem(0)
mpcController.solver.solve(solver.xs[:mpcController.problem.T + 1], solver.us[:mpcController.problem.T])

simulator = AerialSimulator(mpcController.robot_model, mpcController.platform_params, mpcController.dt, solver.xs[0])
t = 0
updateTime = []
solveTime = []
for i in range(0, len(solver.xs)):
    mpcController.problem.x0 = simulator.states[-1]
    start = time.time()
    mpcController.updateProblem(t)
    end = time.time()
    updateTime.append(end - start)
    start = time.time()
    mpcController.solver.solve(mpcController.solver.xs, mpcController.solver.us, mpcController.iters)
    end = time.time()
    solveTime.append(end - start)
    control = np.copy(mpcController.solver.us_squash[0])
    simulator.simulateStep(control)
    t += mpcController.dt

print("Average update time: ", sum(updateTime) / len(updateTime))
print("Average solving time: ", sum(solveTime) / len(solveTime))
