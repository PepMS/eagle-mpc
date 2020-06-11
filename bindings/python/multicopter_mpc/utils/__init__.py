import pinocchio
import crocoddyl
import multicopter_mpc


class TrajectoryGeneratorDerived(multicopter_mpc.OcpAbstract):
    def __init__(self, model, mc_params, dt, mission):
        multicopter_mpc.OcpAbstract.__init__(model, mc_params, dt)
        self.mission = mission

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
