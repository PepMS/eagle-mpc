import sys

import crocoddyl
import multicopter_mpc
import example_robot_data

WITHDISPLAY = 'display' in sys.argv

dt = 10  # ms
useSquash = True
trajectoryName = 'quad_passthrough'

trajectory = multicopter_mpc.Trajectory()
trajectory.autoSetup("/home/pepms/robotics/libraries/multicopter-mpc/config/trajectory/" + trajectoryName + ".yaml")

problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")

if useSquash:
    solver = multicopter_mpc.SolverSbFDDP(problem, trajectory.squash)
else:
    solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=400)

if WITHDISPLAY:
    robot = example_robot_data.load(trajectory.robot_model.name)
    display = crocoddyl.GepettoDisplay(robot)
    display.displayFromSolver(solver)
