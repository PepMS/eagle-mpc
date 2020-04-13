import crocoddyl
import numpy as np


class SolverSquashFDDP():
    def __init__(self, problem, squashingModel):
        # General objects
        self.problem = problem
        self.actuation = self.problem.terminalModel.differential.actuation
        self.state = self.problem.terminalModel.state

        # Squashing
        self.squashingModel = squashingModel  # it could also be retrieved from the actuation model
        self.squashingSmoothInit = 0.1
        self.squashingSmoothMult = 0.5
        self.squashingSmooth = self.squashingSmoothInit
        self.squashingLb = self.squashingModel.s_lb
        self.squashingUb = self.squashingModel.s_ub

        # Barrier
        self.barrierQuadraticWeights = 1.0 / np.power(self.squashingSmooth * (self.squashingUb - self.squashingLb), 2)
        self.barrierWeight = 1e-3
        self.barrierInit()

        # SBFFDP parameters
        self.convergenceInit = 1e-2
        self.convergenceStop = 1e-3
        self.convergenceMult = 1e-1
        self.convergence = self.convergenceInit
        self.maxIters = 100
        self.regInit = 1e-9

        # Inner solvers
        self.solverFDDP = crocoddyl.SolverFDDP(self.problem)
        self.solverFDDP.th_stop_gaps = 1e-3
        self.solverFDDP.setStoppingCriteria(crocoddyl.StoppingType.StopCriteriaCostReduction)
        self.solverFDDP.setStoppingTest(crocoddyl.StoppingTestType.StopTestGaps)
        self.solverDDP = crocoddyl.SolverDDP(self.problem)
        self.solverDDP.setStoppingCriteria(crocoddyl.StoppingType.StopCriteriaCostReduction)

    def barrierInit(self):
        barrierActivationBounds = crocoddyl.ActivationBounds(self.squashingModel.s_lb, self.squashingModel.s_ub)
        barrierActivation = crocoddyl.ActivationModelWeightedQuadraticBarrier(barrierActivationBounds,
                                                                              self.barrierQuadraticWeights)
        squashingBarrierCost = crocoddyl.CostModelControl(self.state, barrierActivation, self.actuation.nu)

        # we only need to change one running model to change them all
        self.problem.runningModels[0].differential.costs.addCost("sBarrier", squashingBarrierCost, self.barrierWeight)

        for idx, m in enumerate(self.problem.runningModels):
            self.problem.updateModel(idx, m)

    def setCallbacks(self, callbacks):
        self.solverFDDP.setCallbacks(callbacks)
        self.solverDDP.setCallbacks(callbacks)

    def solve(self, xsInit=[], ssInit=[], maxIter=100, regInit=1e-9):
        self.xs = xsInit
        self.ss = ssInit

        while (self.convergence >= self.convergenceStop):
            self.squashingUpdate()
            self.barrierUpdate()

            self.solverFDDP.th_stop = self.convergence
            self.solverFDDP.solve(self.xs, self.ss, self.maxIters, False, self.regInit)
            self.xs = self.solverFDDP.xs
            self.ss = self.solverFDDP.us

            self.squashingSmooth *= self.squashingSmoothMult
            self.convergence *= self.convergenceMult

        if not self.solverFDDP.isFeasible:
            self.solverDDP.th_stop = self.solverFDDP.th_stop
            self.solverDDP.solve(self.xs, self.ss, self.maxIters, False, self.regInit)
            self.xs = self.solverDDP.xs
            self.ss = self.solverDDP.ss

    def squashingUpdate(self):
        self.actuation.squashing.smooth = self.squashingSmooth

    def barrierUpdate(self):
        self.barrierQuadraticWeights = 1.0 / np.power(self.squashingSmooth * (self.squashingUb - self.squashingLb), 2)
        # for m in self.problem.runningModels:
        #     m.differential.costs.costs[
        #         'sBarrier'].cost.activation.weights = self.barrierQuadraticWeights
        self.problem.runningModels[0].differential.costs.costs[
            'sBarrier'].cost.activation.weights = self.barrierQuadraticWeights

    def setBarrierWeight(self, weight):
        self.barrierWeight = weight
        # for m in self.problem.runningModels:
        #     m.differential.costs.costs['sBarrier'].weight = self.barrierWeight
        self.problem.runningModels[0].differential.costs.costs['sBarrier'].weight = self.barrierWeight
