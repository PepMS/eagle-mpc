import pinocchio
import crocoddyl

import numpy as np
import math
from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR


def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


class CarrotMpc():
    def __init__(self, robot_model, robot_state, actuation_model, mc_params, mission, state_reference,
                 control_reference, dt):
        self.robot_model = robot_model
        self.robot_state = robot_state
        self.mc_params = mc_params
        self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")
        self.tau_lb = self.mc_params.min_thrust * np.ones(self.mc_params.n_rotors)
        self.tau_ub = self.mc_params.max_thrust * np.ones(self.mc_params.n_rotors)

        self.actuation_model = actuation_model

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
        # self.n_knots = mission.total_knots - 1
        self.n_knots = 100

        self.terminal_weights = [False] * self.n_knots

        # Weights for the costs
        self.w_state_position = np.ones(3)
        self.w_state_orientation = np.ones(3)
        self.w_state_velocity_lin = np.ones(3)
        self.w_state_velocity_ang = np.ones(3)
        self.w_state_running = 1e-5
        self.w_control_running = 5e-2
        self.w_pos_running = 8000.
        self.w_vel_running = 5000.
        self.w_pos_terminal = 10
        self.w_vel_terminal = 10

        self.solver_iters = 70

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

    def createDifferentialModel(self, idx_knot):
        cost_model = crocoddyl.CostModelSum(self.robot_state, self.actuation_model.nu)

        cost_reg_state = self.createCostStateRegularization()
        cost_reg_control = self.createCostControlRegularization(idx_knot)

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

        diff_model = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.robot_state, self.actuation_model,
                                                                      cost_model)

        diff_model.u_lb = self.tau_lb
        diff_model.u_ub = self.tau_ub

        return diff_model

    def createCostStateRegularization(self):
        state_weights = np.hstack(
            (self.w_state_position, self.w_state_orientation, self.w_state_velocity_lin, self.w_state_velocity_ang))

        activation_state = crocoddyl.ActivationModelWeightedQuad(state_weights)
        cost_reg_state = crocoddyl.CostModelState(self.robot_state, activation_state, self.robot_state.zero(),
                                                  self.actuation_model.nu)

        return cost_reg_state

    def createCostControlRegularization(self, idx_knot):

        if idx_knot >= len(self.control_reference):
            control_ref = self.control_reference[-1]
        else:
            control_ref = self.control_reference[idx_knot]

        cost_reg_control = crocoddyl.CostModelControl(self.robot_state, self.actuation_model.nu)
        # cost_reg_control = crocoddyl.CostModelControl(self.robot_state, control_ref)
        return cost_reg_control

    def solve(self, state_trajectory, control_trajectory):
        self.solver.solve(state_trajectory, control_trajectory, self.solver_iters)

    def updateProblem(self, idx_trajectory):
        wp_idxs = self.mission.wp_knot_idx.tolist()
        idx_traj = idx_trajectory
        self.terminal_weights = self.terminal_weights[1:] + [idx_traj in wp_idxs]

        if self.terminal_weights[-1] or idx_traj > len(self.state_reference) - 1:
            wp_idx = len(self.mission.waypoints) - 1
            if idx_traj <= self.mission.total_knots - 1:
                wp_idx = wp_idxs.index(idx_traj)
            self.pose_ref.id = self.frame_base_link_id
            self.pose_ref.placement = self.mission.waypoints[wp_idx].pose
            self.motion_ref.id = self.frame_base_link_id
            self.motion_ref.motion = self.mission.waypoints[wp_idx].velocity
        else:
            self.setReference(idx_traj)

        self.diff_model_terminal.costs.costs['pose_desired'].cost.reference = self.pose_ref
        self.diff_model_terminal.costs.costs['vel_desired'].cost.reference = self.motion_ref

        self.diff_model_terminal.costs.costs['pose_desired'].active = True
        self.diff_model_terminal.costs.costs['vel_desired'].active = True

        # if True in self.terminal_weights and self.terminal_weights[-1] == False:
        #     self.diff_model_terminal.costs.costs['pose_desired'].active = False
        #     self.diff_model_terminal.costs.costs['vel_desired'].active = False
        # else:
        #     self.diff_model_terminal.costs.costs['pose_desired'].active = True
        #     self.diff_model_terminal.costs.costs['vel_desired'].active = True

        idx_traj -= 1
        for idx, model in rev_enumerate(self.diff_models_running):
            if self.terminal_weights[idx]:
                wp_idx = wp_idxs.index(idx_traj)
                self.pose_ref.id = self.frame_base_link_id
                self.pose_ref.placement = self.mission.waypoints[wp_idx].pose
                self.motion_ref.id = self.frame_base_link_id
                self.motion_ref.motion = self.mission.waypoints[wp_idx].velocity
                model.costs.costs['pose_desired'].active = True
                model.costs.costs['vel_desired'].active = True
            else:
                if idx_traj > len(self.state_reference) - 1:
                    model.costs.costs['pose_desired'].active = True
                    model.costs.costs['vel_desired'].active = True
                else:
                    model.costs.costs['pose_desired'].active = False
                    model.costs.costs['vel_desired'].active = False

            model.costs.costs['pose_desired'].cost.reference = self.pose_ref
            model.costs.costs['vel_desired'].cost.reference = self.motion_ref
            idx_traj -= 1

    def setReference(self, idx_trajectory):
        if idx_trajectory >= len(self.state_reference):
            state_ref = self.state_reference[-1]
        else:
            state_ref = self.state_reference[idx_trajectory]
        self.pose_ref.id = self.frame_base_link_id
        self.pose_ref.placement = pinocchio.XYZQUATToSE3(state_ref[:7])
        self.motion_ref.id = self.frame_base_link_id
        self.motion_ref.motion = pinocchio.Motion(state_ref[7:])

    def setStateInitial(self, state):
        self.state_initial = state

        if hasattr(self, 'problem'):
            self.problem.x0 = self.state_initial


class GaussianCarrotMpc():
    def __init__(self, robot_model, robot_state, actuation_model, mc_params, mission, state_reference,
                 control_reference, dt):
        self.robot_model = robot_model
        self.robot_state = robot_state
        self.mc_params = mc_params
        self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")
        self.tau_lb = self.mc_params.min_thrust * np.ones(self.mc_params.n_rotors)
        self.tau_ub = self.mc_params.max_thrust * np.ones(self.mc_params.n_rotors)

        self.actuation_model = actuation_model

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
        # self.n_knots = mission.total_knots - 1
        self.n_knots = 100

        self.terminal_weights = [False] * self.n_knots

        # Weights for the costs
        self.w_state_position = np.ones(3)
        self.w_state_orientation = np.ones(3)
        self.w_state_velocity_lin = np.ones(3)
        self.w_state_velocity_ang = np.ones(3)
        self.w_state_running = 1e-5
        self.w_control_running = 1e-1
        self.w_pos_running = 5000.
        self.w_vel_running = 5000.
        self.w_pos_terminal = 10
        self.w_vel_terminal = 10

        self.solver_iters = 70

    def initializeWPWeights(self):
        self.std_dev = 1

        wp_idxs = self.mission.wp_knot_idx
        self.weight_distribution = [[0] * self.mission.total_knots for _ in range(len(wp_idxs))]
        for idx, wp_idx in enumerate(wp_idxs):
            self.weight_distribution[idx][:wp_idx + 1] = [
                1 / self.std_dev * math.sqrt(1 / (2 * math.pi)) * math.exp(-1 / 2 * ((i - wp_idx) / self.std_dev)**2)
                for i in range(wp_idx + 1)
            ]
        print()

    def createProblem(self):
        self.initializeWPWeights()

        self.diff_model_terminal = self.createDifferentialModel(self.n_knots - 1, True)
        self.int_model_terminal = crocoddyl.IntegratedActionModelEuler(self.diff_model_terminal, 0.)

        for i in reversed(range(self.n_knots - 1)):
            diff_model = self.createDifferentialModel(i, False)
            int_model = crocoddyl.IntegratedActionModelEuler(diff_model, self.dt)
            self.diff_models_running.insert(0, diff_model)
            self.int_models_running.insert(0, int_model)

        self.problem = crocoddyl.ShootingProblem(self.state_initial, self.int_models_running, self.int_model_terminal)
        self.solver = crocoddyl.SolverBoxFDDP(self.problem)

    def createDifferentialModel(self, idx_knot, is_terminal):
        cost_model = crocoddyl.CostModelSum(self.robot_state, self.actuation_model.nu)

        cost_reg_state = self.createCostStateRegularization()
        cost_reg_control = self.createCostControlRegularization()

        cost_model.addCost("state_reg", cost_reg_state, self.w_state_running)
        cost_model.addCost("control_reg", cost_reg_control, self.w_control_running)

        if is_terminal:
            self.setReference(idx_knot)
            cost_pose = crocoddyl.CostModelFramePlacement(self.robot_state, self.pose_ref, self.actuation_model.nu)
            cost_vel = crocoddyl.CostModelFrameVelocity(self.robot_state, self.motion_ref, self.actuation_model.nu)
            cost_model.addCost("pose_desired", cost_pose, self.w_pos_terminal, True)
            cost_model.addCost("vel_desired", cost_vel, self.w_vel_terminal, True)
        else:
            for wp_idx, wp in enumerate(self.mission.waypoints):
                pose_ref = crocoddyl.FramePlacement()
                motion_ref = crocoddyl.FrameMotion()
                pose_ref.id = self.frame_base_link_id
                pose_ref.placement = wp.pose
                motion_ref.id = self.frame_base_link_id
                motion_ref.motion = wp.velocity
                wp_name = "wp" + str(wp_idx)
                cost_pose = crocoddyl.CostModelFramePlacement(self.robot_state, pose_ref, self.actuation_model.nu)
                cost_vel = crocoddyl.CostModelFrameVelocity(self.robot_state, motion_ref, self.actuation_model.nu)

                active = self.weight_distribution[wp_idx][idx_knot] > 1e-8
                w_pos = self.w_pos_running * self.weight_distribution[wp_idx][idx_knot] * active
                w_vel = self.w_vel_running * self.weight_distribution[wp_idx][idx_knot] * active
                cost_model.addCost(wp_name + "_pos", cost_pose, w_pos, active)
                cost_model.addCost(wp_name + "_vel", cost_vel, w_vel, active)

        diff_model = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.robot_state, self.actuation_model,
                                                                      cost_model)

        diff_model.u_lb = self.tau_lb
        diff_model.u_ub = self.tau_ub

        return diff_model

    def createCostStateRegularization(self):
        state_weights = np.hstack(
            (self.w_state_position, self.w_state_orientation, self.w_state_velocity_lin, self.w_state_velocity_ang))

        activation_state = crocoddyl.ActivationModelWeightedQuad(state_weights)
        cost_reg_state = crocoddyl.CostModelState(self.robot_state, activation_state, self.robot_state.zero(),
                                                  self.actuation_model.nu)

        return cost_reg_state

    def createCostControlRegularization(self):
        cost_reg_control = crocoddyl.CostModelControl(self.robot_state, self.actuation_model.nu)

        return cost_reg_control

    def solve(self, state_trajectory, control_trajectory):
        self.solver.solve(state_trajectory, control_trajectory, self.solver_iters)

    def updateProblem(self, idx_trajectory):
        wp_idxs = self.mission.wp_knot_idx.tolist()
        idx_traj = idx_trajectory

        # Set terminal knot
        if idx_traj in wp_idxs:
            wp_idx = wp_idxs.index(idx_traj)
            self.pose_ref.id = self.frame_base_link_id
            self.pose_ref.placement = self.mission.waypoints[wp_idx].pose
            self.motion_ref.id = self.frame_base_link_id
            self.motion_ref.motion = self.mission.waypoints[wp_idx].velocity
        else:
            self.setReference(idx_traj)

        self.diff_model_terminal.costs.costs['pose_desired'].cost.reference = self.pose_ref
        self.diff_model_terminal.costs.costs['vel_desired'].cost.reference = self.motion_ref

        idx_traj -= 1
        for idx, model in rev_enumerate(self.diff_models_running):
            for wp_idx, wp in enumerate(self.mission.waypoints):
                wp_name = "wp" + str(wp_idx)
                if idx_traj >= self.mission.total_knots:
                    active = True
                    w_pos = self.w_pos_running
                    w_vel = self.w_vel_running
                else:
                    active = self.weight_distribution[wp_idx][idx_traj] > 1e-8
                    w_pos = self.w_pos_running * self.weight_distribution[wp_idx][idx_traj] * active / 4
                    w_vel = self.w_vel_running * self.weight_distribution[wp_idx][idx_traj] * active / 4
                if active:
                    asd = 0
                model.costs.costs[wp_name + '_pos'].active = active
                model.costs.costs[wp_name + '_vel'].active = active
                model.costs.costs[wp_name + '_pos'].weight = w_pos
                model.costs.costs[wp_name + '_vel'].weight = w_vel

            idx_traj -= 1

    def setReference(self, idx_trajectory):
        if idx_trajectory >= len(self.state_reference):
            state_ref = self.state_reference[-1]
        else:
            state_ref = self.state_reference[idx_trajectory]
        self.pose_ref.id = self.frame_base_link_id
        self.pose_ref.placement = pinocchio.XYZQUATToSE3(state_ref[:7])
        self.motion_ref.id = self.frame_base_link_id
        self.motion_ref.motion = pinocchio.Motion(state_ref[7:])

    def setStateInitial(self, state):
        self.state_initial = state

        if hasattr(self, 'problem'):
            self.problem.x0 = self.state_initial
