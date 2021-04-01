import numpy as np
import time

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
    def __init__(self,
                 robot,
                 baseParams,
                 rate=-1,
                 freq=1,
                 cameraTF=None,
                 floor=True,
                 frameNames=[],
                 visibility=False,
                 payload=''):
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

        self.payload = payload
        self.payloadGroup = "world/payload"
        self.payloadBoxSize = [0.25, 0.15, 0.1]
        self.payloadSphereSize = 0.05
        self.payloadColor = [153. / 255., 0., 153. / 255., 1.]

        if self.payload != '':
            self._addPayload(self.payload)

    def display(self, xs, us=[], fs=[], ps=[], dts=[], payloads=[], factor=1.):
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
                    if c is False:
                        self.robot.viewer.gui.setVisibility(self.forceGroup + "/" + key, "OFF")
                        self.robot.viewer.gui.setVisibility(self.frictionGroup + "/" + key, "OFF")
                puav = x[:3]
                quav = pinocchio.Quaternion(x[3:7].reshape(4, 1))
                Muav = pinocchio.SE3(quav, puav)
                if us:
                    if i < len(us):
                        u = us[i]
                    else:
                        u = us[-1]

                    for ir, rotor in enumerate(self.baseParams.rotors_pose):
                        R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
                        rotor.rotation = R
                        thrustPose = pinocchio.SE3ToXYZQUAT(Muav * rotor).tolist()
                        thrustName = self.thrustGroup + "/" + self.thrusts[ir]
                        thrustMagnitude = float(u[ir]) / self.thrustRange
                        self.robot.viewer.gui.applyConfiguration(thrustName, thrustPose)
                        self.robot.viewer.gui.setVisibility(thrustName, "ON")
                        self.robot.viewer.gui.resizeArrow(thrustName, self.thrustArrowRadius,
                                                          thrustMagnitude * self.thrustArrowLength)

                if (self.payload == 'box' or self.payload == 'sphere') and payloads:
                    self.robot.viewer.gui.applyConfiguration(self.payloadGroup, payloads[i])

                self.robot.display(x[:self.robot.nq])
                time.sleep(dts[i] * factor)

    def getForceTrajectoryFromSolver(self, solver):
        if len(self.frameTrajNames) == 0:
            return None
        fs = []
        models = solver.problem.runningModels.tolist() + [solver.problem.terminalModel]
        datas = solver.problem.runningDatas.tolist() + [solver.problem.terminalData]
        for i, data in enumerate(datas):
            model = models[i]
            if hasattr(data, "differential"):
                if isinstance(data.differential,
                              crocoddyl.libcrocoddyl_pywrap.DifferentialActionDataContactFwdDynamics):
                    fc = []
                    for key, contact in data.differential.multibody.contacts.contacts.todict().items():
                        if model.differential.contacts.contacts[key].active:
                            oMf = contact.pinocchio.oMi[contact.joint] * contact.jMf
                            fiMo = pinocchio.SE3(contact.pinocchio.oMi[contact.joint].rotation.T,
                                                 contact.jMf.translation)
                            force = fiMo.actInv(contact.f)
                            R = np.eye(3)
                            mu = 0.7
                            for k, c in model.differential.costs.costs.todict().items():
                                if isinstance(c.cost, crocoddyl.libcrocoddyl_pywrap.CostModelContactFrictionCone):
                                    if contact.joint == self.robot.model.frames[c.cost.reference.id].parent:
                                        R = c.cost.reference.cone.R
                                        mu = c.cost.reference.cone.mu
                                        continue
                            fc.append({"key": str(contact.joint), "oMf": oMf, "f": force, "R": R, "mu": mu})
                    fs.append(fc)
                elif isinstance(data.differential, crocoddyl.libcrocoddyl_pywrap.StdVec_DiffActionData):
                    if isinstance(data.differential[0],
                                  crocoddyl.libcrocoddyl_pywrap.DifferentialActionDataContactFwdDynamics):
                        fc = []
                        for key, contact in data.differential[0].multibody.contacts.contacts.todict().items():
                            if model.differential.contacts.contacts[key].active:
                                oMf = contact.pinocchio.oMi[contact.joint] * contact.jMf
                                fiMo = pinocchio.SE3(contact.pinocchio.oMi[contact.joint].rotation.T,
                                                     contact.jMf.translation)
                                force = fiMo.actInv(contact.f)
                                R = np.eye(3)
                                mu = 0.7
                                for k, c in model.differential.costs.costs.todict().items():
                                    if isinstance(c.cost, crocoddyl.libcrocoddyl_pywrap.CostModelContactFrictionCone):
                                        if contact.joint == self.robot.model.frames[c.cost.reference.id].parent:
                                            R = c.cost.reference.cone.R
                                            mu = c.cost.reference.cone.mu
                                            continue
                                fc.append({"key": str(contact.joint), "oMf": oMf, "f": force, "R": R, "mu": mu})
                        fs.append(fc)
            elif isinstance(data, crocoddyl.libcrocoddyl_pywrap.ActionDataImpulseFwdDynamics):
                fc = []
                for key, impulse in data.multibody.impulses.impulses.todict().items():
                    if model.impulses.impulses[key].active:
                        oMf = impulse.pinocchio.oMi[impulse.joint] * impulse.jMf
                        fiMo = pinocchio.SE3(impulse.pinocchio.oMi[impulse.joint].rotation.T, impulse.jMf.translation)
                        force = fiMo.actInv(impulse.f)
                        R = np.eye(3)
                        mu = 0.7
                        for k, c in model.costs.costs.todict().items():
                            if isinstance(c.cost, crocoddyl.libcrocoddyl_pywrap.CostModelContactFrictionCone):
                                if impulse.joint == self.robot.model.frames[c.cost.id].parent:
                                    R = c.cost.cone.R
                                    mu = c.cost.cone.mu
                                    continue
                        fc.append({"key": str(impulse.joint), "oMf": oMf, "f": force, "R": R, "mu": mu})
                fs.append(fc)
        return fs

    def _addThrustArrows(self):
        for thrust in self.thrusts:
            thrustName = self.thrustGroup + "/" + thrust
            self.robot.viewer.gui.addArrow(thrustName, self.thrustArrowRadius, self.thrustArrowLength,
                                           self.thrustArrowColor)
            self.robot.viewer.gui.setFloatProperty(thrustName, "Alpha", 1.)
        if self.fullVisibility:
            self.robot.viewer.gui.setVisibility(self.thrustGroup, "ALWAYS_ON_TOP")

    def _addPayload(self, type):
        if type == 'box':
            self.robot.viewer.gui.addBox(self.payloadGroup, self.payloadBoxSize[0], self.payloadBoxSize[1],
                                         self.payloadBoxSize[2], self.payloadColor)
        elif type == 'sphere':
            self.robot.viewer.gui.addSphere(self.payloadGroup, self.payloadSphereSize, self.payloadColor)
