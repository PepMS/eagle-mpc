import numpy as np
# import math
# import copy
import bisect

import pinocchio
import crocoddyl
# import example_robot_data

import multicopter_mpc


def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


class CarrotMpc(multicopter_mpc.CarrotMpc):
    def __init__(self, trajectory, stateRef, dtRef, yamlPath):
        multicopter_mpc.CarrotMpc.__init__(self, trajectory, stateRef, dtRef, yamlPath)
        self.lastTime = 0

    def createProblem_(self):
        self.ts_ini = []
        for stage in self.trajectory.stages:
            self.ts_ini.append(stage.t_ini)
        # self.solver = multicopter_mpc.SolverSbFDDP(self.problem, self.squash)
        # self.solver.setCallbacks([crocoddyl.CallbackVerbose()])

    def updateProblem(self, currentTime):
        idxStage = self.getActiveStage(currentTime)
        for idx, dam in enumerate(self.dif_models):
            dt = self.dt
            nodeTime = currentTime + idx * dt
            idxStage = self.getActiveStage(nodeTime, idxStage)
            nameStage = self.trajectory.stages[idxStage].name
            for cost in dam.costs.costs.todict():
                if cost[:len(nameStage)] == nameStage or cost == 'barrier':
                    dam.costs.costs[cost].active = True
                else:
                    dam.costs.costs[cost].active = False

            if idx == len(self.dif_models) - 1 and self.trajectory.stages[idxStage].is_transition:
                dam.costs.costs["state"].active = True
                state = self.getStateRef(nodeTime)
                dam.costs.costs["state"].cost.reference = state
        self.lastTime = currentTime

    def getActiveStage(self, time, lastStage=None):
        if lastStage is not None:
            stage = bisect.bisect_right(self.ts_ini, time) - 1
            if stage == lastStage + 2:
                stage -= 1
        else:
            stage = bisect.bisect_right(self.ts_ini, time) - 1

        return stage

    def getStateRef(self, time):
        nq = self.robot_model.nq
        idxState = bisect.bisect_right(self.t_ref, time)
        alpha = (time - self.t_ref[idxState - 1]) / (self.t_ref[idxState] - self.t_ref[idxState - 1])
        q0 = self.state_ref[idxState - 1][:nq]
        q1 = self.state_ref[idxState][:nq]
        qref = pinocchio.interpolate(self.robot_model, q0, q1, alpha)
        stateRef = self.state.zero()
        stateRef[:nq] = qref
        stateRef[nq:] = self.state_ref[idxState - 1][nq:] + alpha * (self.state_ref[idxState][nq:] -
                                                                     self.state_ref[idxState - 1][nq:])
        return stateRef