import numpy as np

import example_robot_data
import pinocchio
import crocoddyl
import multicopter_mpc

from multicopter_mpc.utils.path import MULTICOPTER_MPC_MULTIROTOR_DIR, MULTICOPTER_MPC_MISSION_DIR, MULTICOPTER_MPC_OCP_DIR


class AerialSimulator():
    def __init__(self, dt, x0):
        self.robot = example_robot_data.loadIris()
        self.robot_model = self.robot.model
        self.robot_state = crocoddyl.StateMultibody(self.robot_model)
        self.mc_params = multicopter_mpc.MultiCopterBaseParams()
        self.mc_params.fill(MULTICOPTER_MPC_MULTIROTOR_DIR + "/iris.yaml")
        self.dt = dt

        self.actuation_model = crocoddyl.ActuationModelMultiCopterBase(
            self.robot_state, self.mc_params.n_rotors, self.mc_params.tau_f)
        self.dif_action_model = crocoddyl.DifferentialActionModelFreeFwdDynamics(self.robot_state, self.actuation_model, crocoddyl.CostModelSum(self.robot_state, self.actuation_model.nu))
        self.int_action_model = crocoddyl.IntegratedActionModelRK4(self.dif_action_model, self.dt)
        self.int_action_data = self.int_action_model.createData()

        self.state_initial = x0
        self.states = [x0]
        self.controls = []
    
    def simulateStep(self, u):
        self.controls.append(np.copy(u))
        self.int_action_model.calc(self.int_action_data, self.states[-1], self.controls[-1])
        self.states.append(np.copy(self.int_action_data.xnext))
        
        return self.states[-1]
