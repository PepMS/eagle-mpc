import numpy as np
import math
import copy

import pinocchio
import crocoddyl
import example_robot_data


import multicopter_mpc



def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


class MpcBase():
    def __init__(self):
        parser = multicopter_mpc.ParserYaml("mpc.yaml", "/home/pepms/robotics/libraries/multicopter-mpc/config/mpc")
        self.server = multicopter_mpc.ParamsServer(parser.get_params())

        self.initializeRobotObjects()
        self.loadParams()

    def initializeRobotObjects(self):
        self.robot = example_robot_data.load('hexarotor_370_flying_arm_3')
        self.rModel = self.robot.model
        self.platformParams = multicopter_mpc.MultiCopterBaseParams()
        self.platformParams.autoSetup("robot/platform/", self.server, self.rModel)
        self.rState = crocoddyl.StateMultibody(self.rModel)
        self.actuation = crocoddyl.ActuationModelMultiCopterBase(self.rState, self.platformParams.n_rotors,
                                                                 self.platformParams.tau_f)
        self.squash = crocoddyl.SquashingModelSmoothSat(self.platformParams.u_lb, self.platformParams.u_ub,
                                                        self.actuation.nu)
        self.actSquash = crocoddyl.ActuationSquashingModel(self.actuation, self.squash, self.actuation.nu)

    def loadParams(self):
        self.integrationMethod = "Euler"
        self.solver = "SbFDDP"
        self.knots = 100
        self.iters = 4


class CarrotMpc(MpcBase):
    def __init__(self, trajectory):
        MpcBase.__init__(self)
        self.trajectory = trajectory
        self.createProblem()

    def createProblem(self):
        self.costs = crocoddyl.CostModelSum(self.rState, self.actuation.nu)
        for stage in self.trajectory.stages:
            for cost_name in stage.costs.costs.todict():
                cost = copy.deepcopy(stage.costs.costs[cost_name])
                cost_cp = stage.costs.costs[cost_name]
                print()

