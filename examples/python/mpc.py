import numpy as np
import crocoddyl
import multicopter_mpc

from mpc_controllers import CarrotMpc
from simulator import AerialSimulator

# Trajectory
trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/quad_passthrough.yaml")

dt = 10
problem = trajectory.createProblem(dt, True, "IntegratedActionModelEuler")

solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

mpcController = CarrotMpc(trajectory, solver.xs, dt,
                          "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc/mpc.yaml")
mpcController.createProblem_()
mpcController.updateProblem(0)
mpcController.solver.solve(solver.xs[:mpcController.problem.T + 1], solver.us[:mpcController.problem.T])

simulator = AerialSimulator(mpcController.robot_model, mpcController.platform_params, mpcController.dt, solver.xs[0])
time = 0
for i in range(0, len(solver.xs)):
    mpcController.problem.x0 = simulator.states[-1]
    mpcController.updateProblem(time)
    mpcController.solver.solve(mpcController.solver.xs, mpcController.solver.us)
    control = np.copy(mpcController.solver.us_squash[0])
    simulator.simulateStep(control)
    time += mpcController.dt