import crocoddyl
import multicopter_mpc

from mpc_controllers import CarrotMpc

# Trajectory
trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/am_eagle_catch.yaml")
x0 = trajectory.state.zero()
problem = trajectory.createProblem(10, True, "IntegratedActionModelEuler")
solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

mpcController = CarrotMpc(trajectory)

print()