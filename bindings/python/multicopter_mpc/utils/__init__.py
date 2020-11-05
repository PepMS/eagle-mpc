import pinocchio
import crocoddyl
import multicopter_mpc
import example_robot_data

import numpy as np
from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR, MULTICOPTER_MPC_OCP_DIR


def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


class TrajectoryGeneratorDerived(multicopter_mpc.OcpAbstract):
    def __init__(self, model, mc_params, dt, mission):
        multicopter_mpc.OcpAbstract.__init__(model, mc_params, dt)
        self.mission = mission

    def setParameters(self):
        self.w_state_position = np.ones(3)
        self.w_state_orientation = np.ones(3)
        self.w_state_velocity_lin = np.ones(3)
        self.w_state_velocity_ang = np.ones(3)
        self.w_state_running = 1e-6
        self.w_control_running = 1e-4
        self.w_pos_running = 1e-2
        self.w_vel_running = 1e-2
        self.w_pos_terminal = 100
        self.w_vel_terminal = 10

    def createProblem(self, solver_type):
        self.n_knots = self.mission.total_knots

        cost_reg_state = self.createCostStateRegularization()
        cost_reg_control = self.createCostControlRegularization()

        for idx_wp, wp in enumerate(self.mission.waypoints):
            cost_model_running = crocoddyl.CostModelSum(self.state, self.actuation.nu)
            cost_model_terminal = crocoddyl.CostModelSum(self.state, self.actuation.nu)

            cost_model_running.addCost("x_reg", cost_reg_state, 1e-6)
            cost_model_running.addCost("u_ref", cost_reg_control, 1e-4)

            if idx_wp < len(self.mission.waypoints) - 1:
                cost_model_terminal.addCost("x_reg", cost_reg_state, 1e-6)
                cost_model_terminal.addCost("u_ref", cost_reg_control, 1e-4)

            frame_ref = crocoddyl.FramePlacement(self.base_link_id, wp.pose)
            cost_goal = crocoddyl.CostModelFramePlacement(self.state, frame_ref, self.actuation.nu)
            cost_model_running.addCost("track_pos_cost", cost_goal, 1e-2)
            cost_model_running.addCost("goal_pos_cost", cost_goal, 100)

            if wp.velocity is not None:
                vel_ref = crocoddyl.FrameMotion(self.base_link_id, wp.velocity)
                cost_goal_vel = crocoddyl.CostModelFrameVelocity(self.state, vel_ref, self.actuation.nu)
                cost_model_running.addCost("track_vel_cost", cost_goal_vel, 1e-2)
                cost_model_running.addCost("goal_vel_cost", cost_goal_vel, 100)

            diff_model_running = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.state, self.actuation,
                                                                                  cost_model_running)
            diff_model_terminal = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.state, self.actuation,
                                                                                   cost_model_terminal)
            int_model_running = crocoddyl.IntegratedActionModelEuler(diff_model_running, self.dt)
            int_model_terminal = crocoddyl.IntegratedActionModelEuler(diff_model_terminal, self.dt)

            if idx_wp < len(self.mission.waypoints) - 1:
                int_model_running.u_lb = self.u_lb
                int_model_running.u_ub = self.u_ub
                int_model_terminal.u_lb = self.u_lb
                int_model_terminal.u_ub = self.u_ub

                diff_models_running = [diff_model_running for i in range(wp.knots - 1)]
                diff_models_running.append(diff_model_terminal)
                self.diff_models_running.extend(diff_models_running)

                int_models_running = [int_model_running for i in range(wp.knots - 1)]
                int_models_running.append(int_model_terminal)
                self.int_models_running.extend(int_models_running)
            else:
                int_model_running.u_lb = self.u_lb
                int_model_running.u_ub = self.u_ub

                diff_models_running = [diff_model_running for i in range(wp.knots - 1)]
                self.diff_models_running.extend(diff_models_running)
                self.diff_model_terminal = diff_model_terminal

                int_models_running = [int_model_running for i in range(wp.knots - 1)]
                self.int_models_running.extend(int_models_running)
                self.int_model_terminal = int_model_terminal

        self.problem = crocoddyl.ShootingProblem(self.mission.x0, self.int_models_running, self.int_model_terminal)

    def createCostStateRegularization(self):
        state_weights = pinocchio.utils.zero(self.state.ndx)

        state_weights[:3] = 1
        state_weights[3:6] = 1
        state_weights[self.model.nv:self.model.nv + 3] = 1
        state_weights[self.model.nv + 3:self.model.nv + 6] = 1000

        activation_state = crocoddyl.ActivationModelWeightedQuad(state_weights)

        cost_reg_state = crocoddyl.CosModelState(self.state, activation_state, self.state.zero(), self.actuation.nu)

        return cost_reg_state

    def createCostControlRegularization(self):
        cost_reg_control = crocoddyl.CostModelControl(self.state, self.actuation.nu)
        return cost_reg_control

    def setSolver(self, solver_type):
        if solver_type == multicopter_mpc.SolverType.SolverTypeBoxFDDP:
            self.solver = crocoddyl.SolverBoxFDDP(self.problem)
        elif solver_type == multicopter_mpc.SolverType.SolverTypeSquashBoxFDDP:
            print("Solver Squash not available")


class CarrotMpc():
    def __init__(self, mission, state_reference, control_reference, dt):
        self.robot = example_robot_data.loadIris()
        self.robot_model = self.robot.model
        self.robot_state = crocoddyl.StateMultibody(self.robot_model)
        self.mc_params = multicopter_mpc.MultiCopterBaseParams()
        self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")
        self.tau_lb = self.mc_params.min_thrust * np.ones(self.mc_params.n_rotors)
        self.tau_ub = self.mc_params.max_thrust * np.ones(self.mc_params.n_rotors)

        self.actuation_model = crocoddyl.ActuationModelMultiCopterBase(
            self.robot_state, self.mc_params.n_rotors, self.mc_params.tau_f)

        self.dt = dt
        self.diff_models_running = []
        self.int_models_running = []

        self.frame_base_link_id = self.robot_model.getFrameId(self.mc_params.base_link_name)
        self.pose_ref = crocoddyl.FramePlacement()
        self.motion_ref = crocoddyl.FrameMotion()

        self.mission = mission
        self.state_reference = state_reference
        self.control_reference = control_reference
        self.state_initial = self.state_reference[0]
        self.n_knots = 100

        self.terminal_weights = [False] * self.n_knots

        # Weights for the costs
        self.w_state_position = np.ones(3)
        self.w_state_orientation = np.ones(3)
        self.w_state_velocity_lin = np.ones(3)
        self.w_state_velocity_ang = np.ones(3)
        self.w_state_running = 1e-5
        self.w_control_running = 1e-2
        self.w_pos_running = 2500.
        self.w_vel_running = 2500.
        self.w_pos_terminal = 10
        self.w_vel_terminal = 10

        self.solver_iters = 5

    def initializeTerminalWeights(self):
        wp_idxs = self.mission.wp_knot_idx
        last_wp_idx = 0

        while last_wp_idx < len(wp_idxs.tolist()) and self.n_knots - 1 >= wp_idxs[last_wp_idx]:
            self.terminal_weights[wp_idxs[last_wp_idx]] = True
            last_wp_idx += 1

        if last_wp_idx >= len(wp_idxs.tolist()):
            self.terminal_weights[wp_idxs[last_wp_idx - 1]:] = True

    def createProblem(self):
        self.initializeTerminalWeights()

        self.diff_model_terminal = self.createDifferentialModel(self.n_knots - 1)
        self.int_model_terminal = crocoddyl.IntegratedActionModelEuler(self.diff_model_terminal, 0.)

        for i in reversed(range(self.n_knots - 1)):
            diff_model = self.createDifferentialModel(i)
            int_model = crocoddyl.IntegratedActionModelEuler(diff_model, self.dt)
            self.diff_models_running.insert(0, diff_model)
            self.int_models_running.insert(0, int_model)

        self.problem = crocoddyl.ShootingProblem(self.state_initial, self.int_models_running, self.int_model_terminal)
        self.solver = crocoddyl.SolverBoxFDDP(self.problem)
        self.solver.setCallbacks([crocoddyl.CallbackVerbose()])

        # to implement:
        # -> append the terminal model into the running models
        # -> set solver

    def createDifferentialModel(self, idx_knot):
        cost_model = crocoddyl.CostModelSum(self.robot_state, self.actuation_model.nu)

        cost_reg_state = self.createCostStateRegularization()
        cost_reg_control = self.createCostControlRegularization()

        cost_model.addCost("state_reg", cost_reg_state, self.w_state_running)
        cost_model.addCost("control_reg", cost_reg_control, self.w_control_running)

        if idx_knot == self.n_knots - 1 or self.terminal_weights[idx_knot]:
            self.setReference(idx_knot)

        w_pos = self.w_pos_running
        w_vel = self.w_vel_running
        if idx_knot == self.n_knots - 1:
            w_pos = self.w_pos_terminal
            w_vel = self.w_vel_terminal

        cost_pose = crocoddyl.CostModelFramePlacement(self.robot_state, self.pose_ref, self.actuation_model.nu)
        cost_vel = crocoddyl.CostModelFrameVelocity(self.robot_state, self.motion_ref, self.actuation_model.nu)

        if self.terminal_weights[idx_knot] or idx_knot == self.n_knots - 1:
            cost_model.addCost("pose_desired", cost_pose, w_pos, True)
            cost_model.addCost("vel_desired", cost_vel, w_vel, True)
        else:
            cost_model.addCost("pose_desired", cost_pose, w_pos, False)
            cost_model.addCost("vel_desired", cost_vel, w_vel, False)

        diff_model = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.robot_state, self.actuation_model, cost_model)

        diff_model.u_lb = self.tau_lb
        diff_model.u_ub = self.tau_ub

        return diff_model

    def createCostStateRegularization(self):
        state_weights = np.hstack((self.w_state_position, self.w_state_orientation,
                                   self.w_state_velocity_lin, self.w_state_velocity_ang))

        activation_state = crocoddyl.ActivationModelWeightedQuad(state_weights)
        cost_reg_state = crocoddyl.CostModelState(
            self.robot_state, activation_state, self.robot_state.zero(), self.actuation_model.nu)

        return cost_reg_state

    def createCostControlRegularization(self):
        cost_reg_control = crocoddyl.CostModelControl(self.robot_state, self.actuation_model.nu)

        return cost_reg_control

    def solve(self, state_trajectory, control_trajectory):
        self.solver.solve(state_trajectory, control_trajectory, self.solver_iters)

    def updateProblem(self, idx_trajectory):
        wp_idxs = self.mission.wp_knot_idx.tolist()
        idx_traj = idx_trajectory
        self.terminal_weights = self.terminal_weights[1:] + [idx_traj in wp_idxs]

        if self.terminal_weights[-1]:
            wp_idx = wp_idxs.index(idx_traj)
            self.pose_ref.id = self.frame_base_link_id
            self.pose_ref = self.mission.waypoints[wp_idx].pose
            self.motion_ref.id = self.frame_base_link_id
            self.motion_ref.motion = self.mission.waypoints[wp_idx].vel
        else:
            self.setReference(idx_traj)

        self.diff_model_terminal.costs.costs['pose_desired'].cost.reference = self.pose_ref
        self.diff_model_terminal.costs.costs['vel_desired'].cost.reference = self.motion_ref

        self.diff_model_terminal.costs.costs['pose_desired'].active = True
        self.diff_model_terminal.costs.costs['vel_desired'].active = True

        idx_traj -= 1
        for idx, model in rev_enumerate(self.diff_models_running):
            if self.terminal_weights[idx]:
                wp_idx = wp_idxs.index(idx_traj)
                self.pose_ref.id = self.frame_base_link_id
                self.pose_ref = self.mission.waypoints[wp_idx].pose
                self.motion_ref.id = self.frame_base_link_id
                self.motion_ref.motion = self.mission.waypoints[wp_idx].vel
                model.costs.costs['pose_desired'].active = True
                model.costs.costs['vel_desired'].active = True
            else:
                if idx_trajectory > len(self.state_reference) - 1:
                    model.costs.costs['pose_desired'].active = True
                    model.costs.costs['vel_desired'].active = True
                else:
                    model.costs.costs['pose_desired'].active = False
                    model.costs.costs['vel_desired'].active = False

            model.costs.costs['pose_desired'].cost.reference = self.pose_ref
            model.costs.costs['vel_desired'].cost.reference = self.motion_ref

            idx_traj -= 1

    def setReference(self, idx_trajectory):
        state_ref = self.state_reference[idx_trajectory]
        self.pose_ref.id = self.frame_base_link_id
        self.pose_ref.placement = pinocchio.XYZQUATToSE3(state_ref[:7])
        self.motion_ref.id = self.frame_base_link_id
        self.motion_ref.motion = pinocchio.Motion(state_ref[7:])

    def setStateInitial(self, state):
        self.state_initial = state

        if hasattr(self, 'problem'):
            self.problem.x0 = self.state_initial


class MpcMain():
    def __init__(self, controller):
        self.controller = controller
        self.state = self.controller.robot_state.zero()
        self.controller.setStateInitial(self.state)
        self.controller.createProblem()
        self.controller.solve(self.controller.state_reference[:self.controller.n_knots],
                              self.controller.control_reference[:self.controller.n_knots - 1])
        self.state_trajectory = self.controller.solver.xs
        self.control_trajectory = self.controller.solver.us

        self.trajectory_cursor = self.controller.n_knots - 1

    def runMpcStep(self):
        self.controller.state_initial
        self.controller.solve(self.state_trajectory, self.control_trajectory)
        self.state_trajectory[:-1] = self.controller.solver.xs[1:]
        self.control_trajectory[:-1] = self.controller.solver.us[1:]
        self.trajectory_cursor += 1
        self.state_trajectory[-1] = self.controller.state_reference[self.trajectory_cursor]
        self.control_trajectory[-1] = self.control_trajectory[-2]

        self.controller.updateProblem(self.trajectory_cursor)
