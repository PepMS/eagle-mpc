import sys

import crocoddyl
import example_robot_data

import eagle_mpc
from eagle_mpc.utils.path import EAGLE_MPC_YAML_DIR

WITHDISPLAY = 'display' in sys.argv

dt = 20  # ms
useSquash = True
robotName = 'hexacopter370_flying_arm_3'
trajectoryName = 'displacement'

trajectory = eagle_mpc.Trajectory()
trajectory.autoSetup(EAGLE_MPC_YAML_DIR + "/" + robotName + "/trajectories/" + trajectoryName + ".yaml")
problem = trajectory.createProblem(dt, useSquash, "IntegratedActionModelEuler")

if useSquash:
    solver = eagle_mpc.SolverSbFDDP(problem, trajectory.squash)
else:
    solver = crocoddyl.SolverBoxFDDP(problem)

solver.setCallbacks([crocoddyl.CallbackVerbose()])
solver.solve([], [], maxiter=100)

if WITHDISPLAY:
    robot = example_robot_data.load(trajectory.robot_model.name)
    display = crocoddyl.GepettoDisplay(robot)
    display.displayFromSolver(solver)
