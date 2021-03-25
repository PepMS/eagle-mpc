import numpy as np
import time
import copy

import pinocchio
import crocoddyl


def rotationMatrixFromTwoVectors(a, b):
    a_copy = a / np.linalg.norm(a)
    b_copy = b / np.linalg.norm(b)
    a_cross_b = np.cross(a_copy, b_copy, axis=0)
    s = np.linalg.norm(a_cross_b)

    if s == 0:
        return np.eye(3)
    c = np.dot(a_copy, b_copy)
    ab_skew = pinocchio.skew(a_cross_b)
    return np.eye(3) + ab_skew + np.dot(ab_skew, ab_skew) * (1 - c) / s**2


class MulticopterMpcDisplay(crocoddyl.GepettoDisplay):
    def __init__(self, robot, baseParams, rate=-1, freq=1, cameraTF=None, floor=True, frameNames=[], visibility=False):
        crocoddyl.GepettoDisplay.__init__(self, robot, rate, freq, cameraTF, floor, frameNames, visibility)

        self.baseParams = baseParams
        self.thrustGroup = "world/robot/thrusts"
        self.thrusts = ["rotor" + str(i) for i in range(self.baseParams.n_rotors)]
        self.thrustRange = self.baseParams.max_thrust - self.baseParams.min_thrust
        self.thrustArrowRadius = 0.01
        self.thrustArrowLength = 0.5
        self.thrustArrowColor = [1., 0., 1., 1.]

        self.robot.viewer.gui.createGroup(self.thrustGroup)
        self._addThrustArrows()

    def display(self, xs, us=[], fs=[], ps=[], dts=[], factor=1.):
        if ps:
            for key, p in ps.items():
                self.robot.viewer.gui.setCurvePoints(self.frameTrajGroup + "/" + key, p)
        if not dts:
            dts = [0.] * len(xs)

        S = 1 if self.rate <= 0 else max(len(xs) / self.rate, 1)
        for i, x in enumerate(xs):
            if not i % S:
                if fs:
                    self.activeContacts = {k: False for k, c in self.activeContacts.items()}
                    for f in fs[i]:
                        key = f["key"]
                        pose = f["oMf"]
                        wrench = f["f"]
                        # Display the contact forces
                        R = rotationMatrixFromTwoVectors(self.x_axis, wrench.linear)
                        forcePose = pinocchio.SE3ToXYZQUATtuple(pinocchio.SE3(R, pose.translation))
                        forceMagnitud = np.linalg.norm(wrench.linear) / self.totalWeight
                        forceName = self.forceGroup + "/" + key
                        self.robot.viewer.gui.setVector3Property(forceName, "Scale", [1. * forceMagnitud, 1., 1.])
                        self.robot.viewer.gui.applyConfiguration(forceName, forcePose)
                        self.robot.viewer.gui.setVisibility(forceName, "ON")
                        # Display the friction cones
                        position = pose
                        position.rotation = f["R"]
                        frictionName = self.frictionGroup + "/" + key
                        self._setConeMu(key, f["mu"])
                        self.robot.viewer.gui.applyConfiguration(
                            frictionName, list(np.array(pinocchio.SE3ToXYZQUAT(position)).squeeze()))
                        self.robot.viewer.gui.setVisibility(frictionName, "ON")
                        self.activeContacts[key] = True
                for key, c in self.activeContacts.items():
                    if c == False:
                        self.robot.viewer.gui.setVisibility(self.forceGroup + "/" + key, "OFF")
                        self.robot.viewer.gui.setVisibility(self.frictionGroup + "/" + key, "OFF")
                puav = x[:3]
                quav = pinocchio.Quaternion(x[3:7].reshape(4, 1))
                Muav = pinocchio.SE3(quav, puav)
                if us and i < len(us):
                    for ir, rotor in enumerate(self.baseParams.rotors_pose):
                        R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
                        rotor.rotation = R
                        thrustPose = pinocchio.SE3ToXYZQUAT(Muav * rotor).tolist()
                        thrustName = self.thrustGroup + "/" + self.thrusts[ir]
                        thrustMagnitude = float(us[i][ir]) / self.thrustRange
                        self.robot.viewer.gui.applyConfiguration(thrustName, thrustPose)
                        self.robot.viewer.gui.setVisibility(thrustName, "ON")
                        self.robot.viewer.gui.resizeArrow(thrustName, self.thrustArrowRadius,
                                                          thrustMagnitude * self.thrustArrowLength)
                self.robot.display(x[:self.robot.nq])
                time.sleep(dts[i] * factor)

    def _addThrustArrows(self):
        for thrust in self.thrusts:
            thrustName = self.thrustGroup + "/" + thrust
            self.robot.viewer.gui.addArrow(thrustName, self.thrustArrowRadius, self.thrustArrowLength,
                                           self.thrustArrowColor)
            self.robot.viewer.gui.setFloatProperty(thrustName, "Alpha", 1.)
        if self.fullVisibility:
            self.robot.viewer.gui.setVisibility(self.thrustGroup, "ALWAYS_ON_TOP")