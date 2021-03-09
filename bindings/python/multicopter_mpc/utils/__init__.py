import pinocchio
import crocoddyl
import multicopter_mpc
import example_robot_data

import numpy as np
from multicopter_mpc.utils.mpc_controllers import CarrotMpc


def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


# class TrajectoryGeneratorDerived(multicopter_mpc.OcpAbstract):
#     def __init__(self, model, mc_params, dt, mission):
#         multicopter_mpc.OcpAbstract.__init__(model, mc_params, dt)
#         self.mission = mission

#     def setParameters(self):
#         self.w_state_position = np.ones(3)
#         self.w_state_orientation = np.ones(3)
#         self.w_state_velocity_lin = np.ones(3)
#         self.w_state_velocity_ang = np.ones(3)
#         self.w_state_running = 1e-6
#         self.w_control_running = 1e-4
#         self.w_pos_running = 1e-2
#         self.w_vel_running = 1e-2
#         self.w_pos_terminal = 100
#         self.w_vel_terminal = 10

#     def createProblem(self, solver_type):
#         self.n_knots = self.mission.total_knots

#         cost_reg_state = self.createCostStateRegularization()
#         cost_reg_control = self.createCostControlRegularization()

#         for idx_wp, wp in enumerate(self.mission.waypoints):
#             cost_model_running = crocoddyl.CostModelSum(self.state, self.actuation.nu)
#             cost_model_terminal = crocoddyl.CostModelSum(self.state, self.actuation.nu)

#             cost_model_running.addCost("x_reg", cost_reg_state, 1e-6)
#             cost_model_running.addCost("u_ref", cost_reg_control, 1e-4)

#             if idx_wp < len(self.mission.waypoints) - 1:
#                 cost_model_terminal.addCost("x_reg", cost_reg_state, 1e-6)
#                 cost_model_terminal.addCost("u_ref", cost_reg_control, 1e-4)

#             frame_ref = crocoddyl.FramePlacement(self.base_link_id, wp.pose)
#             cost_goal = crocoddyl.CostModelFramePlacement(self.state, frame_ref, self.actuation.nu)
#             cost_model_running.addCost("track_pos_cost", cost_goal, 1e-2)
#             cost_model_running.addCost("goal_pos_cost", cost_goal, 100)

#             if wp.velocity is not None:
#                 vel_ref = crocoddyl.FrameMotion(self.base_link_id, wp.velocity)
#                 cost_goal_vel = crocoddyl.CostModelFrameVelocity(self.state, vel_ref, self.actuation.nu)
#                 cost_model_running.addCost("track_vel_cost", cost_goal_vel, 1e-2)
#                 cost_model_running.addCost("goal_vel_cost", cost_goal_vel, 100)

#             diff_model_running = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.state, self.actuation,
#                                                                                   cost_model_running)
#             diff_model_terminal = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.state, self.actuation,
#                                                                                    cost_model_terminal)
#             int_model_running = crocoddyl.IntegratedActionModelEuler(diff_model_running, self.dt)
#             int_model_terminal = crocoddyl.IntegratedActionModelEuler(diff_model_terminal, self.dt)

#             if idx_wp < len(self.mission.waypoints) - 1:
#                 int_model_running.u_lb = self.u_lb
#                 int_model_running.u_ub = self.u_ub
#                 int_model_terminal.u_lb = self.u_lb
#                 int_model_terminal.u_ub = self.u_ub

#                 diff_models_running = [diff_model_running for i in range(wp.knots - 1)]
#                 diff_models_running.append(diff_model_terminal)
#                 self.diff_models_running.extend(diff_models_running)

#                 int_models_running = [int_model_running for i in range(wp.knots - 1)]
#                 int_models_running.append(int_model_terminal)
#                 self.int_models_running.extend(int_models_running)
#             else:
#                 int_model_running.u_lb = self.u_lb
#                 int_model_running.u_ub = self.u_ub

#                 diff_models_running = [diff_model_running for i in range(wp.knots - 1)]
#                 self.diff_models_running.extend(diff_models_running)
#                 self.diff_model_terminal = diff_model_terminal

#                 int_models_running = [int_model_running for i in range(wp.knots - 1)]
#                 self.int_models_running.extend(int_models_running)
#                 self.int_model_terminal = int_model_terminal

#         self.problem = crocoddyl.ShootingProblem(self.mission.x0, self.int_models_running, self.int_model_terminal)

#     def createCostStateRegularization(self):
#         state_weights = pinocchio.utils.zero(self.state.ndx)

#         state_weights[:3] = 1
#         state_weights[3:6] = 1
#         state_weights[self.model.nv:self.model.nv + 3] = 1
#         state_weights[self.model.nv + 3:self.model.nv + 6] = 1000

#         activation_state = crocoddyl.ActivationModelWeightedQuad(state_weights)

#         cost_reg_state = crocoddyl.CosModelState(self.state, activation_state, self.state.zero(), self.actuation.nu)

#         return cost_reg_state

#     def createCostControlRegularization(self):
#         cost_reg_control = crocoddyl.CostModelControl(self.state, self.actuation.nu)
#         return cost_reg_control

#     def setSolver(self, solver_type):
#         if solver_type == multicopter_mpc.SolverType.SolverTypeBoxFDDP:
#             self.solver = crocoddyl.SolverBoxFDDP(self.problem)
#         elif solver_type == multicopter_mpc.SolverType.SolverTypeSquashBoxFDDP:
#             print("Solver Squash not available")


# class RecedingMPC():
#     def __init__(self, robot_model, robot_state, actuation_model, mc_params, mission, state_reference,
#                  control_reference, dt):
#         self.robot_model = robot_model
#         self.robot_state = robot_state
#         self.mc_params = mc_params
#         self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")
#         self.tau_lb = self.mc_params.min_thrust * np.ones(self.mc_params.n_rotors)
#         self.tau_ub = self.mc_params.max_thrust * np.ones(self.mc_params.n_rotors)

#         self.actuation_model = actuation_model

#         self.dt = dt
#         self.diff_models_running = []
#         self.int_models_running = []

#         self.frame_base_link_id = self.robot_model.getFrameId(self.mc_params.base_link_name)
#         self.pose_ref = crocoddyl.FramePlacement()
#         self.motion_ref = crocoddyl.FrameMotion()

#         self.mission = mission
#         self.state_reference = state_reference
#         self.control_reference = control_reference
#         self.state_initial = self.state_reference[0]
#         self.n_knots = self.mission.total_knots

#         # Weights for the costs
#         self.w_state_position = np.ones(3)
#         self.w_state_orientation = np.ones(3)
#         self.w_state_velocity_lin = np.ones(3)
#         self.w_state_velocity_ang = np.ones(3)
#         self.w_state_running = 1e-5
#         self.w_control_running = 1e-2
#         self.w_pos_running = 2500.
#         self.w_vel_running = 2500.
#         self.w_pos_terminal = 10
#         self.w_vel_terminal = 10

#         self.solver_iters = 70

#         self.counter = 1

#     def createProblem(self):
#         cost_reg_state = self.createCostStateRegularization()
#         cost_reg_control = self.createCostControlRegularization()

#         for idx_wp, wp in enumerate(self.mission.waypoints):
#             cost_model_running = crocoddyl.CostModelSum(self.robot_state, self.actuation_model.nu)
#             cost_model_terminal = crocoddyl.CostModelSum(self.robot_state, self.actuation_model.nu)

#             cost_model_running.addCost("x_reg", cost_reg_state, self.w_state_running)
#             cost_model_running.addCost("u_ref", cost_reg_control, self.w_control_running)

#             w_pos = self.w_pos_terminal
#             w_vel = self.w_vel_terminal
#             if idx_wp < len(self.mission.waypoints) - 1:
#                 cost_model_terminal.addCost("x_reg", cost_reg_state, self.w_state_running)
#                 cost_model_terminal.addCost("u_ref", cost_reg_control, self.w_control_running)
#                 w_pos = self.w_pos_terminal / self.dt
#                 w_vel = self.w_vel_terminal / self.dt

#             frame_ref = crocoddyl.FramePlacement(self.frame_base_link_id, wp.pose)
#             cost_goal = crocoddyl.CostModelFramePlacement(self.robot_state, frame_ref, self.actuation_model.nu)
#             cost_model_terminal.addCost("pos_desired", cost_goal, w_pos)

#             if wp.velocity is not None:
#                 vel_ref = crocoddyl.FrameMotion(self.frame_base_link_id, wp.velocity)
#                 cost_goal_vel = crocoddyl.CostModelFrameVelocity(self.robot_state, vel_ref, self.actuation_model.nu)
#                 cost_model_terminal.addCost("goal_vel_cost", cost_goal_vel, w_vel)

#             diff_model_running = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.robot_state,
#                                                                                   self.actuation_model,
#                                                                                   cost_model_running)
#             diff_model_terminal = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.robot_state,
#                                                                                    self.actuation_model,
#                                                                                    cost_model_terminal)
#             int_model_running = crocoddyl.IntegratedActionModelEuler(diff_model_running, self.dt)

#             n_run_knots = 0
#             if idx_wp == 0:
#                 n_run_knots = wp.knots - 1
#             else:
#                 n_run_knots = wp.knots - 2

#             if idx_wp < len(self.mission.waypoints) - 1:
#                 int_model_running.u_lb = self.tau_lb
#                 int_model_running.u_ub = self.tau_ub

#                 diff_models_running = [diff_model_running for i in range(n_run_knots)]
#                 diff_models_running.append(diff_model_terminal)
#                 self.diff_models_running.extend(diff_models_running)

#                 int_models_running = [int_model_running for i in range(n_run_knots)]
#                 int_model_terminal = crocoddyl.IntegratedActionModelEuler(diff_model_terminal, self.dt)
#                 int_model_terminal.u_lb = self.tau_lb
#                 int_model_terminal.u_ub = self.tau_ub

#                 int_models_running.append(int_model_terminal)
#                 self.int_models_running.extend(int_models_running)
#             else:
#                 int_model_running.u_lb = self.tau_lb
#                 int_model_running.u_ub = self.tau_ub

#                 diff_models_running = [diff_model_running for i in range(n_run_knots)]
#                 self.diff_models_running.extend(diff_models_running)
#                 self.diff_model_terminal = diff_model_terminal

#                 int_models_running = [int_model_running for i in range(n_run_knots)]
#                 self.int_models_running.extend(int_models_running)
#                 int_model_terminal = crocoddyl.IntegratedActionModelEuler(diff_model_terminal, 0.)
#                 self.int_model_terminal = int_model_terminal

#         self.problem = crocoddyl.ShootingProblem(self.mission.x0, self.int_models_running, self.int_model_terminal)
#         self.solver = crocoddyl.SolverBoxFDDP(self.problem)
#         # self.solver.setCallbacks([crocoddyl.CallbackVerbose()])

#     def createCostStateRegularization(self):
#         state_weights = np.hstack(
#             (self.w_state_position, self.w_state_orientation, self.w_state_velocity_lin, self.w_state_velocity_ang))

#         activation_state = crocoddyl.ActivationModelWeightedQuad(state_weights)
#         cost_reg_state = crocoddyl.CostModelState(self.robot_state, activation_state, self.robot_state.zero(),
#                                                   self.actuation_model.nu)

#         return cost_reg_state

#     def createCostControlRegularization(self):
#         cost_reg_control = crocoddyl.CostModelControl(self.robot_state, self.actuation_model.nu)

#         return cost_reg_control

#     def solve(self, state_trajectory, control_trajectory):
#         self.solver.solve(state_trajectory, control_trajectory, self.solver_iters)

#     def updateProblem(self, idx_trajectory):
#         self.problem = crocoddyl.ShootingProblem(self.state_initial, self.int_models_running[self.counter:],
#                                                  self.int_model_terminal)
#         # self.problem.runningModels = self.problem.runningModels[1:]
#         self.solver = crocoddyl.SolverBoxFDDP(self.problem)
#         # self.solver.setCallbacks([crocoddyl.CallbackVerbose()])

#         self.counter += 1

#     def setStateInitial(self, state):
#         self.state_initial = state

#         if hasattr(self, 'problem'):
#             self.problem.x0 = self.state_initial


# class MpcMain():
#     def __init__(self, mission_name, controller_type):
#         self.robot = example_robot_data.loadIris()
#         self.robot_model = self.robot.model
#         self.robot_state = crocoddyl.StateMultibody(self.robot_model)
#         self.mc_params = multicopter_mpc.MultiCopterBaseParams()
#         self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")
#         self.actuation_model = crocoddyl.ActuationModelMultiCopterBase(self.robot_state, self.mc_params.n_rotors,
#                                                                        self.mc_params.tau_f)
#         self.mission_name = mission_name
#         self.mission = multicopter_mpc.Mission(self.robot.nq + self.robot.nv)
#         self.mission.fillWaypoints(MULTICOPTER_MPC_MISSION_DIR + "/" + mission_name)

#         self.dt = 4e-3
#         self.initializeTrajectoryGenerator()
#         self.controller_type = controller_type
#         if controller_type == "carrot":
#             self.controller = CarrotMpc(self.robot_model, self.robot_state, self.actuation_model,
#                                         self.mc_params, self.mission,
#                                         self.trajectory.getStateTrajectory(0, self.trajectory.n_knots - 1),
#                                         self.trajectory.getControlTrajectory(0, self.trajectory.n_knots - 2), self.dt)
#         elif controller_type == "receding":
#             self.controller = RecedingMPC(self.robot_model, self.robot_state, self.actuation_model, self.mc_params,
#                                           self.mission,
#                                           self.trajectory.getStateTrajectory(0, self.trajectory.n_knots - 1),
#                                           self.trajectory.getControlTrajectory(0,
#                                                                                self.trajectory.n_knots - 2), self.dt)
#         elif controller_type == "gaussian_carrot":
#             self.controller = GaussianCarrotMpc(self.robot_model, self.robot_state, self.actuation_model,
#                                                 self.mc_params, self.mission,
#                                                 self.trajectory.getStateTrajectory(0, self.trajectory.n_knots - 1),
#                                                 self.trajectory.getControlTrajectory(0, self.trajectory.n_knots - 2),
#                                                 self.dt)

#         self.state = self.robot_state.zero()
#         self.controller.setStateInitial(self.state)
#         self.controller.createProblem()
#         self.controller.solve(self.controller.state_reference[:self.controller.n_knots],
#                               self.controller.control_reference[:self.controller.n_knots - 1])
#         self.state_trajectory = self.controller.solver.xs
#         self.control_trajectory = self.controller.solver.us

#         self.trajectory_cursor = self.controller.n_knots - 1

#     def initializeTrajectoryGenerator(self):
#         self.trajectory = multicopter_mpc.TrajectoryGenerator(self.robot_model, self.mc_params, self.mission)
#         self.trajectory.loadParameters(MULTICOPTER_MPC_OCP_DIR + "/trajectory-generator.yaml")
#         self.trajectory.createProblem(multicopter_mpc.SolverType.SolverTypeBoxFDDP,
#                                       multicopter_mpc.IntegratorType.IntegratorTypeEuler, self.dt)
#         self.trajectory.setSolverCallbacks(True)
#         state_guess = self.mission.interpolateTrajectory("cold")
#         control = pinocchio.utils.zero(4)
#         control_guess = [control for _ in range(0, len(state_guess) - 1)]
#         self.trajectory.solve(state_guess, control_guess)

#     def runMpcStep(self):
#         self.controller.setStateInitial(self.state)
#         if self.controller_type == "receding":
#             self.controller.solve(self.state_trajectory, self.control_trajectory)
#             control = self.controller.solver.us[0]
#             # self.state_trajectory = []
#             self.state_trajectory = self.controller.solver.xs[1:]
#             # self.control_trajectory = []
#             self.control_trajectory = self.controller.solver.us[1:]
#             self.trajectory_cursor += 1
#         else:
#             self.controller.solve(self.state_trajectory, self.control_trajectory)
#             self.state_trajectory[:-1] = self.controller.solver.xs[1:]
#             self.control_trajectory[:-1] = self.controller.solver.us[1:]
#             self.trajectory_cursor += 1
#             if self.trajectory_cursor >= len(self.controller.state_reference):
#                 state_hover = self.robot_state.zero()
#                 state_hover[:3] = self.controller.state_reference[-1][:3]
#                 quat_hover = pinocchio.Quaternion(self.controller.state_reference[-1][6], 0., 0.,
#                                                   self.controller.state_reference[-1][5])
#                 quat_hover.normalize()
#                 state_hover[5] = quat_hover.z
#                 state_hover[6] = quat_hover.w
#                 self.state_trajectory[-1] = state_hover
#             else:
#                 self.state_trajectory[-1] = self.controller.state_reference[self.trajectory_cursor]
#             self.control_trajectory[-1] = self.control_trajectory[-2]

#         control = np.copy(self.controller.solver.us[0])
#         self.controller.updateProblem(self.trajectory_cursor)

#         return control
