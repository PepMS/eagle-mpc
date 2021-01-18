import numpy as np

import pinocchio


class multicopterParams:
    def __init__(self, nRotors):
        self.nRotors = nRotors
        self.tauF = np.zeros([6, self.nRotors])

        self.maxThrust = 0
        self.minThrust = 0
        self.cm = 0
        self.cf = 0

        self.maxTorque = 0
        self.minTorque = 0


class Hexarotor680Params(multicopterParams):
    def __init__(self):
        multicopterParams.__init__(self, 6)

        diameter = 0.4064
        airDensity = 1.22
        cT0 = 0.062
        cP0 = 0.02
        
        self.cf = (cT0 * airDensity * (diameter)**4) / (2 * np.pi)**2
        moment_constant = cP0 * diameter / (cT0 * 2 * np.pi)
        self.cm = moment_constant * self.cf
        self.minThrust = 0.0
        self.maxThrust = 2.53 * 9.81  # Tarot 4008 with 1660 propellers
        self.minTorque = -3
        self.maxTorque = 3

        self.lArm = 0.68 / 2
        self.M_rot_base = []
        rotorsAngle = [1 * np.pi / 6, 3 * np.pi / 6, 5 * np.pi / 6, 7 * np.pi / 6, 9 * np.pi / 6, 11 * np.pi / 6]

        for idx, angle in enumerate(rotorsAngle):
            R = np.identity(3)
            R[:2, :2] = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            pos = np.dot(R, np.array([self.lArm, 0, 0]))
            M = pinocchio.SE3(R, pos)
            self.M_rot_base.append(M)
            self.tauF[:3, idx] = np.dot(M.rotation, np.array([0, 0, 1]))
            self.tauF[3:, idx] = np.cross(M.translation, np.dot(M.rotation, np.array(
                [0, 0, 1]))) + (-1)**idx * self.cm / self.cf * np.dot(M.rotation, np.array([0, 0, 1]))


class Hexarotor370Params(multicopterParams):
    def __init__(self):
        multicopterParams.__init__(self, 6)

        diameter = 7 * 0.0254
        airDensity = 1.22
        cT0 = 0.134
        cP0 = 0.08
        
        self.cf = (cT0 * airDensity * (diameter)**4) / (2 * np.pi)**2
        moment_constant = cP0 * diameter / (cT0 * 2 * np.pi)
        self.cm = moment_constant * self.cf
        self.minThrust = 0.0
        self.maxThrust = 2.11 * 9.81 # T-Motor F90 1300KV, 6S, GF7042
        self.minTorque = -1
        self.maxTorque = 1

        self.lArm = 0.370 / 2
        self.M_rot_base = []
        rotorsAngle = [1 * np.pi / 6, 3 * np.pi / 6, 5 * np.pi / 6, 7 * np.pi / 6, 9 * np.pi / 6, 11 * np.pi / 6]

        for idx, angle in enumerate(rotorsAngle):
            R = np.identity(3)
            R[:2, :2] = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
            pos = np.dot(R, np.array([self.lArm, 0, 0]))
            M = pinocchio.SE3(R, pos)
            self.M_rot_base.append(M)
            self.tauF[:3, idx] = np.dot(M.rotation, np.array([0, 0, 1]))
            self.tauF[3:, idx] = np.cross(M.translation, np.dot(M.rotation, np.array(
                [0, 0, 1]))) + (-1)**idx * self.cm / self.cf * np.dot(M.rotation, np.array([0, 0, 1]))
        
        print()