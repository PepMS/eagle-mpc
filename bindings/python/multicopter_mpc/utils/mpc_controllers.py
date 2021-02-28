# import numpy as np
# import math
# import copy
import bisect

# import pinocchio
# import crocoddyl
# import example_robot_data

import multicopter_mpc


def rev_enumerate(lname):
    return reversed(list(enumerate(lname)))


class CarrotMpc(multicopter_mpc.CarrotMpc):
    def __init__(self, trajectory, yaml_path):
        multicopter_mpc.CarrotMpc.__init__(self, trajectory, yaml_path)

    def createProblem_(self):
        self.ts_ini = []
        for stage in self.trajectory.stages:
            self.ts_ini.append(stage.t_ini)

    def updateProblem(self, currentTime):
        idxStage = self.getActiveStage(currentTime)
        for idx, dam in enumerate(self.dif_models):
            dt = 10
            nodeTime = currentTime + idx * dt
            idxStage = self.getActiveStage(nodeTime, idxStage)
            nameStage = self.trajectory.stages[idxStage].name
            for cost in dam.costs.costs.todict():
                if cost[:len(nameStage)] == nameStage:
                    dam.costs.costs[cost].active = True
                else:
                    dam.costs.costs[cost].active = False
            # activate terminal node with reference

    def getActiveStage(self, time, lastStage=None):
        if lastStage is not None:
            stage = bisect.bisect_right(self.ts_ini, time) - 1
            if stage == lastStage + 2:
                stage -= 1
        else:
            stage = bisect.bisect_right(self.ts_ini, time) - 1

        return stage
