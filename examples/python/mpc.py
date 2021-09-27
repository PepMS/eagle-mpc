import numpy as np
import time
import copy
import crocoddyl

import eagle_mpc
from eagle_mpc.utils.path import EAGLE_MPC_YAML_DIR
from eagle_mpc.utils.simulator import AerialSimulator
from eagle_mpc.utils.plots import PlotControlsGroup, showPlots

# Trajectory
dt = 20  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
# trajectoryName = 'eagle_catch_nc'
trajectoryName = 'displacement'
mpcName = 'carrot'

trajectory = eagle_mpc.Trajectory()
trajectory.autoSetup(EAGLE_MPC_YAML_DIR + "/" + robotName + "/trajectories/" + trajectoryName + ".yaml")
problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")

if useSquash:
    solver = eagle_mpc.SolverSbFDDP(problem, trajectory.squash)
else:
    solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

mpcPath = EAGLE_MPC_YAML_DIR + "/" + robotName + "/mpc/mpc.yaml"
if mpcName == 'rail':
    mpcController = eagle_mpc.RailMpc(solver.xs, dt, mpcPath)
elif mpcName == 'weighted':
    mpcController = eagle_mpc.WeightedMpc(trajectory, dt, mpcPath)
else:
    mpcController = eagle_mpc.CarrotMpc(trajectory, solver.xs, dt, mpcPath)

mpcController.updateProblem(0)
mpcController.solver.solve(solver.xs[:mpcController.problem.T + 1], solver.us[:mpcController.problem.T])
mpcController.solver.convergence_init = 1e-3

dtSimulator = 2
simulator = AerialSimulator(mpcController.robot_model, mpcController.platform_params, dtSimulator, solver.xs[0])
t = 0
updateTime = []
solveTime = []

for i in range(0, int(problem.T * dt * 1.2)):
    mpcController.problem.x0 = simulator.states[-1]
    start = time.time()
    mpcController.updateProblem(int(t))
    end = time.time()
    updateTime.append(end - start)
    start = time.time()
    mpcController.solver.solve(mpcController.solver.xs, mpcController.solver.us, mpcController.iters)
    end = time.time()
    solveTime.append(end - start)
    control = np.copy(mpcController.solver.us_squash[0])
    simulator.simulateStep(control)
    t += dtSimulator

time = np.array([i * dt / 1000.0 for i in range(len(solver.us))])
us_plot_tg = np.vstack(solver.us_squash).T
PlotControlsGroup(us_plot_tg, time, 6)

time_mpc = np.array([i * dt / 1000.0 for i in range(len(simulator.controls))])
us_plot_mpc = np.vstack(simulator.controls).T
PlotControlsGroup(us_plot_mpc, time_mpc, 6)
# showPlots()

# print(simulator.states[-1])
print("Average update time: ", sum(updateTime) / len(updateTime))
print("Average solving time: ", sum(solveTime) / len(solveTime))
