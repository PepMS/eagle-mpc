import numpy as np
import time
import copy
import crocoddyl

import multicopter_mpc
from multicopter_mpc.utils.simulator import AerialSimulator
from multicopter_mpc.utils.plots import PlotControlsGroup, showPlots

# Trajectory
dt = 20  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
# trajectoryName = 'eagle_catch_nc'
trajectoryName = 'displacement'
mpcName = 'carrot'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/trajectories/" + robotName +
                     '_' + trajectoryName + ".yaml")

problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")
if useSquash:
    solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
else:
    solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

if mpcName == 'rail':
    mpcController = multicopter_mpc.RailMpc(
        solver.xs, dt,
        "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/mpc/" + robotName + "_mpc.yaml")
elif mpcName == 'weighted':
    mpcController = multicopter_mpc.WeightedMpc(
        trajectory, dt,
        "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/mpc/" + robotName + "_mpc.yaml")
else:
    mpcController = multicopter_mpc.CarrotMpc(
        trajectory, solver.xs, dt,
        "/home/pepms/wsros/mpc-ws/src/multicopter_mpc/multicopter_mpc_yaml/mpc/" + robotName + "_mpc.yaml")

mpcController.updateProblem(0)
mpcController.solver.solve(solver.xs[:mpcController.problem.T + 1], solver.us[:mpcController.problem.T])
mpcController.solver.convergence_init = 1e-3
# mpcController.solver.th_stop_gaps = 1e-2

dtSimulator = 2
simulator = AerialSimulator(mpcController.robot_model, mpcController.platform_params, dtSimulator, solver.xs[0])
t = 0
updateTime = []
solveTime = []
# for i in range(0, problem.T * dt * 2):
# for i in range(0, problem.T * dt):
while t <= 5400:
    mpcController.problem.x0 = simulator.states[-1]
    start = time.time()
    # At t=4330 it turns carrot terminal model to false
    mpcController.updateProblem(int(t))
    end = time.time()
    updateTime.append(end - start)
    start = time.time()
    print("Time stamp: ", t)
    mpcController.solver.solve(mpcController.solver.xs, mpcController.solver.us, mpcController.iters)
    end = time.time()
    solveTime.append(end - start)
    control = np.copy(mpcController.solver.us_squash[0])
    simulator.simulateStep(control)
    t += dtSimulator
    if t > 4330:
        print()

time = np.array([i * dt / 1000.0 for i in range(len(solver.us))])
us_plot_tg = np.vstack(solver.us_squash).T
PlotControlsGroup(us_plot_tg, time, 6)

time_mpc = np.array([i * dt / 1000.0 for i in range(len(simulator.controls))])
us_plot_mpc = np.vstack(simulator.controls).T
PlotControlsGroup(us_plot_mpc, time_mpc, 6)
showPlots()

# print(simulator.states[-1])
# print("Average update time: ", sum(updateTime) / len(updateTime))
# print("Average solving time: ", sum(solveTime) / len(solveTime))
