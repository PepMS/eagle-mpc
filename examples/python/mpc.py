import crocoddyl
import multicopter_mpc

from mpc_controllers import CarrotMpc

# Trajectory
trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/quad_passthrough.yaml")
x0 = trajectory.state.zero()
problem = trajectory.createProblem(10, True, "IntegratedActionModelEuler")
solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

mpcController = CarrotMpc(trajectory, "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc/mpc.yaml")
mpcController.createProblem_()
mpcController.updateProblem(2410)

print()