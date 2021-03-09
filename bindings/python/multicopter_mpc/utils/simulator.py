import numpy as np

import crocoddyl
import multicopter_mpc

class AerialSimulator():
    def __init__(self, robotModel, platformParams, dt, x0):
        self.robotModel = robotModel
        self.robotState = crocoddyl.StateMultibody(self.robotModel)
        self.platformParams = platformParams
        self.dt = dt / 1000.

        self.actuationModel = crocoddyl.ActuationModelMultiCopterBase(self.robotState, self.platformParams.n_rotors,
                                                                      self.platformParams.tau_f)
        self.difAM = crocoddyl.DifferentialActionModelFreeFwdDynamics(
            self.robotState, self.actuationModel, crocoddyl.CostModelSum(self.robotState, self.actuationModel.nu))
        self.intAM = crocoddyl.IntegratedActionModelRK4(self.difAM, self.dt)
        self.intAD = self.intAM.createData()

        self.x0 = x0
        self.states = [x0]
        self.controls = []

    def simulateStep(self, u):
        self.controls.append(np.copy(u))
        self.intAM.calc(self.intAD, self.states[-1], self.controls[-1])
        self.states.append(np.copy(self.intAD.xnext))

        return self.states[-1]
