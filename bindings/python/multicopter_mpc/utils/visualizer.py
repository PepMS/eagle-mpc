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


class UAVUAMVisualizer():
    def __init__(self, solver, robot, rotorsPlacement, thrustMin, thrustMax, floor=True, payload=False, frames=[]):
        self.solver = solver
        self.robot = robot
        self.fullVisibility = False
        self.payload = payload
        self.rate = -1

        # UAV
        self.rotorsPlacement = rotorsPlacement
        self.nRotors = len(self.rotorsPlacement)
        self.thrustGroup = "world/robot/thrusts"
        self.thrusts = ["rotor" + str(i) for i in range(self.nRotors)]
        self.thrustMin = thrustMin
        self.thrustMax = thrustMax
        self.thrustRange = self.thrustMax - self.thrustMin

        # Walls
        self.wallCount = 0
        self.wallColor = [0, 0.1686, 0.2196, 1]

        # Visuals properties
        self.floorGroup = "world/floor"
        self.floorScale = [0.5, 0.5, 0.5]
        self.floorColor = [0.7, 0.7, 0.7, 1.]
        self.wallGroup = "world/wall"
        self.backgroundColor = [1., 1., 1., 1.]
        self.thrustArrowRadius = 0.01
        self.thrustArrowLength = 0.5
        self.thrustArrowColor = [1., 0., 1., 1.]

        # CoG
        self.cogGroup = "world/robot/cog"
        self.cogColor = [0., 0., 204. / 255., 1.]
        self.cogRadius = 0.03

        # Payload
        self.payloadGroup = "world/sensor_box"
        self.payloadSize = [0.25, 0.15, 0.1]
        self.payloadColor = [153. / 255., 0., 153. / 255., 1.]
        self.payloadCog_link3 = np.array([0, 0, 0.05])

        # Frames
        self.frames = frames
        self.framesGroup = "world/robot/frames"
        self.framesXYZSize = 0.1
        self.framesSphere = 0.02
        self.framesColor = [204. / 255., 0., 0., 1.]

        # Contact Forces
        self.forceGroup = "world/robot/contact_forces"
        self.forceRadius = 0.015
        self.forceLength = 0.5
        self.forceColor = [1., 0., 1., 1.]

        # Friction Cones
        self.frictionGroup = "world/robot/friction_cone"
        self.frictionConeScale = 0.2
        self.frictionConeRays = True
        self.frictionConeColor1 = [0., 0.4, 0.79, 0.5]
        self.frictionConeColor2 = [0., 0.4, 0.79, 0.5]

        # Contacts
        self.activeContacts = {}
        self.frictionMu = {}
        for n in frames:
            if n != "base_link":
                parentId = self.robot.model.frames[robot.model.getFrameId(n)].parent
                self.activeContacts[str(parentId)] = True
                self.frictionMu[str(parentId)] = 0.7

        self.addRobot()
        self.setBackground()
        if floor:
            self.addFloor()
        self.totalWeight = sum(m.mass
                               for m in self.robot.model.inertias) * np.linalg.norm(self.robot.model.gravity.linear)

        self.x_axis = np.array([1., 0., 0.])
        self.z_axis = np.array([0., 0., 1.])

        self.robot.viewer.gui.createGroup(self.thrustGroup)
        self.robot.viewer.gui.createGroup(self.wallGroup)
        self.robot.viewer.gui.createGroup(self.framesGroup)
        self.robot.viewer.gui.createGroup(self.forceGroup)
        self.robot.viewer.gui.createGroup(self.frictionGroup)

        self.addCog()
        self.addThrustArrows()
        if self.payload:
            self.addPayload()
        if self.frames is not None:
            self.addFrames()
        self.addForceArrows()
        self.addFrictionCones()

    def display(self, xs, us=[], dts=[], factor=1., payloads=[]):
        if not dts:
            dts = [0.] * len(xs)

        if self.payload:
            if not payloads:
                payloads = [True] * len(xs)
            payloadMs = self.getPayloadTrajectory(xs)
        else:
            payloads = [False] * len(xs)

        cogs = self.getCogTrajectory(xs, payloads)

        if self.frames:
            framesMs = self.getFramesTrajectory(xs)
            forces = self.getForceTrajectory()

        S = 1 if self.rate <= 0 else max(len(xs) / self.rate, 1)
        for i, x in enumerate(xs):
            if not i % S:
                if forces:
                    self.activeContacts = {k: False for k, c in self.activeContacts.items()}
                    for f in forces[i]:
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
                        position.rotation = rotationMatrixFromTwoVectors(self.z_axis, f["nsurf"])
                        frictionName = self.frictionGroup + "/" + key
                        # self._setConeMu(key, f["mu"])
                        self.robot.viewer.gui.applyConfiguration(
                            frictionName, list(np.array(pinocchio.SE3ToXYZQUAT(position)).squeeze()))
                        self.robot.viewer.gui.setVisibility(frictionName, "ON")
                        self.activeContacts[key] = True
                puav = x[:3]
                quav = pinocchio.Quaternion(x[3:7].reshape(4, 1))
                Muav = pinocchio.SE3(quav, puav)
                if us and i < len(us):
                    for ir, rotor in enumerate(self.rotorsPlacement):
                        R = np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]])
                        Mrotor = pinocchio.SE3(R, rotor)
                        thrustPose = pinocchio.SE3ToXYZQUAT(Muav * Mrotor).tolist()
                        thrustName = self.thrustGroup + "/" + self.thrusts[ir]
                        thrustMagnitude = float(us[i][ir]) / self.thrustRange
                        self.robot.viewer.gui.applyConfiguration(thrustName, thrustPose)
                        self.robot.viewer.gui.setVisibility(thrustName, "ON")
                        self.robot.viewer.gui.resizeArrow(thrustName, self.thrustArrowRadius,
                                                          thrustMagnitude * self.thrustArrowLength)
                if self.payload:
                    if payloads[i]:
                        last_M_w_box = copy.deepcopy(payloadMs[i])
                    self.robot.viewer.gui.applyConfiguration(self.payloadGroup,
                                                             pinocchio.SE3ToXYZQUAT(last_M_w_box).tolist())

                if self.frames is not None:
                    for frame in self.frames:
                        self.robot.viewer.gui.applyConfiguration(self.framesGroup + "/" + frame,
                                                                 pinocchio.SE3ToXYZQUAT(framesMs[frame][i]).tolist())

                self.robot.display(x[:self.robot.nq])
                self.robot.viewer.gui.applyConfiguration(self.cogGroup, cogs[i].tolist() + [1, 0, 0, 0])

                time.sleep(dts[i] * factor)

    def addRobot(self):
        # Spawn robot model
        self.robot.initViewer(windowName="crocoddyl", loadModel=False)
        self.robot.loadViewerModel(rootNodeName="robot")

    def setBackground(self):
        # Set white background and floor
        window_id = self.robot.viewer.gui.getWindowID("crocoddyl")
        self.robot.viewer.gui.setBackgroundColor1(window_id, self.backgroundColor)
        self.robot.viewer.gui.setBackgroundColor2(window_id, self.backgroundColor)

    def addFloor(self):
        self.robot.viewer.gui.createGroup(self.floorGroup)
        self.robot.viewer.gui.addFloor(self.floorGroup + "/flat")
        self.robot.viewer.gui.setScale(self.floorGroup + "/flat", self.floorScale)
        self.robot.viewer.gui.setColor(self.floorGroup + "/flat", self.floorColor)
        self.robot.viewer.gui.setLightingMode(self.floorGroup + "/flat", "OFF")

    def addThrustArrows(self):
        for thrust in self.thrusts:
            thrustName = self.thrustGroup + "/" + thrust
            self.robot.viewer.gui.addArrow(thrustName, self.thrustArrowRadius, self.thrustArrowLength,
                                           self.thrustArrowColor)
            self.robot.viewer.gui.setFloatProperty(thrustName, "Alpha", 1.)
        if self.fullVisibility:
            self.robot.viewer.gui.setVisibility(self.thrustGroup, "ALWAYS_ON_TOP")

    def addCog(self):
        self.robot.viewer.gui.addSphere(self.cogGroup, self.cogRadius, self.cogColor)

    def addPayload(self):
        self.robot.viewer.gui.addBox("world/sensor_box", 0.25, 0.15, 0.1, [153 / 255, 0, 153 / 255, 1.])

    def addWall(self, start, end):
        self.wallCount = self.wallCount + 1
        wallName = self.wallGroup + "/wall" + str(self.wallCount)
        corner1 = (start[0], start[1], start[2])
        corner2 = (start[0], start[1], end[2])
        corner3 = (end[0], end[1], end[2])
        corner4 = (end[0], end[1], start[2])
        self.robot.viewer.gui.addSquareFace(wallName, corner1, corner2, corner3, corner4, self.wallColor)
        self.robot.viewer.gui.setFloatProperty(wallName, "Alpha", 0.6)

    def addFrames(self):
        for frame in self.frames:
            self.robot.viewer.gui.addXYZaxis(self.framesGroup + "/" + frame, self.framesColor, self.framesSphere,
                                             self.framesXYZSize)
            self.robot.viewer.gui.refresh()

    def addForceArrows(self):
        for key in self.activeContacts:
            forceName = self.forceGroup + "/" + key
            self.robot.viewer.gui.addArrow(forceName, self.forceRadius, self.forceLength, self.forceColor)
            self.robot.viewer.gui.setFloatProperty(forceName, "Alpha", 1.)
        if self.fullVisibility:
            self.robot.viewer.gui.setVisibility(self.forceGroup, "ALWAYS_ON_TOP")

    def addFrictionCones(self):
        for key in self.activeContacts:
            self.createCone(key, self.frictionConeScale, mu=0.7)

    def createCone(self, coneName, scale=1., mu=0.7):
        m_generatrices = np.matrix(np.empty([3, 4]))
        m_generatrices[:, 0] = np.matrix([mu, mu, 1.]).T
        m_generatrices[:, 0] = m_generatrices[:, 0] / np.linalg.norm(m_generatrices[:, 0])
        m_generatrices[:, 1] = m_generatrices[:, 0]
        m_generatrices[0, 1] *= -1.
        m_generatrices[:, 2] = m_generatrices[:, 0]
        m_generatrices[:2, 2] *= -1.
        m_generatrices[:, 3] = m_generatrices[:, 0]
        m_generatrices[1, 3] *= -1.
        generatrices = m_generatrices

        v = [[0., 0., 0.]]
        for k in range(m_generatrices.shape[1]):
            v.append(m_generatrices[:3, k].T.tolist()[0])
        v.append(m_generatrices[:3, 0].T.tolist()[0])
        coneGroup = self.frictionGroup + "/" + coneName
        self.robot.viewer.gui.createGroup(coneGroup)

        meshGroup = coneGroup + "/cone"
        result = self.robot.viewer.gui.addCurve(meshGroup, v, self.frictionConeColor1)
        self.robot.viewer.gui.setCurveMode(meshGroup, 'TRIANGLE_FAN')
        if self.frictionConeRays:
            lineGroup = coneGroup + "/lines"
            self.robot.viewer.gui.createGroup(lineGroup)
            for k in range(m_generatrices.shape[1]):
                l = self.robot.viewer.gui.addLine(lineGroup + "/" + str(k), [0., 0., 0.],
                                                  m_generatrices[:3, k].T.tolist()[0], self.frictionConeColor2)
        self.robot.viewer.gui.setScale(coneGroup, [scale, scale, scale])
        if self.fullVisibility:
            self.robot.viewer.gui.setVisibility(meshGroup, "ALWAYS_ON_TOP")
            self.robot.viewer.gui.setVisibility(lineGroup, "ALWAYS_ON_TOP")

    def getCogTrajectory(self, xs, payloads):
        cogs = []
        for x, payload in zip(xs, payloads):
            self.changeRobotModel(payload)
            pinocchio.centerOfMass(self.robot.model, self.robot.data, x[:self.robot.model.nq], False)
            cogs.append(np.copy(self.robot.data.com[0]))

        return cogs

    def getPayloadTrajectory(self, xs):
        payloadMs = []
        for x in xs:
            q = x[:self.robot.nq]
            pinocchio.framesForwardKinematics(self.robot.model, self.robot.data, q)
            M_w_link3 = self.robot.data.oMf[self.robot.model.getFrameId("arm_link3")]
            cogPayload_w = M_w_link3.act(self.payloadCog_link3)
            M_w_box = pinocchio.SE3(M_w_link3.rotation, cogPayload_w)
            payloadMs.append(copy.deepcopy(M_w_box))

        return payloadMs

    def getFramesTrajectory(self, xs):
        ps = {fr: [] for fr in self.frames}

        for x in xs:
            pinocchio.framesForwardKinematics(self.robot.model, self.robot.data, x[:self.robot.nq])
            for frame in self.frames:
                ps[frame].append(copy.deepcopy(self.robot.data.oMf[self.robot.model.getFrameId(frame)]))

        return ps

    def getForceTrajectory(self):
        if len(self.frames) == 0:
            return None
        fs = []
        solver = self.solver
        models = solver.problem.runningModels.tolist() + [solver.problem.terminalModel]
        datas = solver.problem.runningDatas.tolist() + [solver.problem.terminalData]
        for i, data in enumerate(datas):
            model = models[i]
            if hasattr(data, "differential"):
                if isinstance(data.differential, crocoddyl.DifferentialActionDataContactFwdDynamics):
                    fc = []
                    for key, contact in data.differential.multibody.contacts.contacts.todict().items():
                        if model.differential.contacts.contacts[key].active:
                            oMf = contact.pinocchio.oMi[contact.joint] * contact.jMf
                            fiMo = pinocchio.SE3(contact.pinocchio.oMi[contact.joint].rotation.T,
                                                 contact.jMf.translation)
                            force = fiMo.actInv(contact.f)
                            nsurf = np.array([0., 0., 1.])
                            mu = 0.7
                            for k, c in model.differential.costs.costs.todict().items():
                                if isinstance(c.cost, crocoddyl.CostModelContactFrictionCone):
                                    if contact.joint == self.robot.model.frames[c.cost.reference.id].parent:
                                        nsurf = c.cost.reference.cone.nsurf
                                        mu = c.cost.reference.cone.mu
                                        continue
                            fc.append({"key": str(contact.joint), "oMf": oMf, "f": force, "nsurf": nsurf, "mu": mu})
                    fs.append(fc)
            elif isinstance(data, crocoddyl.ActionDataImpulseFwdDynamics):
                fc = []
                for key, impulse in data.multibody.impulses.impulses.todict().items():
                    if model.impulses.impulses[key].active:
                        oMf = impulse.pinocchio.oMi[impulse.joint] * impulse.jMf
                        fiMo = pinocchio.SE3(impulse.pinocchio.oMi[impulse.joint].rotation.T, impulse.jMf.translation)
                        force = fiMo.actInv(impulse.f)
                        nsurf = np.array([0., 0., 1.])
                        mu = 0.7
                        for k, c in model.costs.costs.todict().items():
                            if isinstance(c.cost, crocoddyl.CostModelContactFrictionCone):
                                if impulse.joint == self.robot.model.frames[c.cost.id].parent:
                                    nsurf = c.cost.cone.nsurf
                                    mu = c.cost.cone.mu
                                    continue
                        fc.append({"key": str(impulse.joint), "oMf": oMf, "f": force, "nsurf": nsurf, "mu": mu})
                fs.append(fc)
        return fs

    def changeRobotModel(self, payload):
        a = 2
        # if payload:
        #     self.robot.model.inertias[self.robot.model.getJointId('arm_link1_arm_link2')].mass = 1.16984
        #     self.robot.model.inertias[self.robot.model.getJointId('arm_link1_arm_link2')].lever = np.array(
        #         [-0.00262794, -2.51267e-10, 0.178323])
        #     self.robot.model.inertias[self.robot.model.getJointId('arm_link1_arm_link2')].inertia = np.array(
        #         [[0.00629506, -4.24279e-12, -0.000205871], [-4.24279e-12, 0.00966053, -2.83174e-11],
        #          [-0.000205871, -2.83174e-11, 0.00715303]])

        # else:
        #     self.robot.model.inertias[self.robot.model.getJointId('arm_link1_arm_link2')].mass = 0.16984
        #     self.robot.model.inertias[self.robot.model.getJointId('arm_link1_arm_link2')].lever = np.array(
        #         [-0.018101, -1.7307e-09, 0.10957])
        #     self.robot.model.inertias[self.robot.model.getJointId('arm_link1_arm_link2')].inertia = np.array(
        #         [[0.00014755, 3.0539e-13, 5.4946e-06], [3.0539e-13, 0.00013211, -8.108e-12],
        #          [5.4946e-06, -8.108e-12, 2.2127e-05]])