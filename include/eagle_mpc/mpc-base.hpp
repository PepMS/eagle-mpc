///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_MPC_BASE_HPP_
#define EAGLE_MPC_MPC_BASE_HPP_

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "crocoddyl/core/action-base.hpp"
#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"
#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/cost-base.hpp"
#include "crocoddyl/core/diff-action-base.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/solvers/fddp.hpp"
#include "crocoddyl/core/solvers/box-ddp.hpp"
#include "crocoddyl/core/solvers/box-fddp.hpp"
#include "crocoddyl/core/utils/callbacks.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "eagle_mpc/factory/cost.hpp"
#include "eagle_mpc/factory/int-action.hpp"
#include "eagle_mpc/utils/params_server.hpp"
#include "eagle_mpc/sbfddp.hpp"

namespace eagle_mpc
{
enum class SolverTypes { SolverSbFDDP, SolverBoxFDDP, SolverBoxDDP };

static std::map<std::string, SolverTypes> SolverTypes_init_map()
{
    std::map<std::string, SolverTypes> m;
    m.clear();
    m.insert({"SolverSbFDDP", SolverTypes::SolverSbFDDP});
    m.insert({"SolverBoxFDDP", SolverTypes::SolverBoxFDDP});
    m.insert({"SolverBoxDDP", SolverTypes::SolverBoxDDP});
    return m;
}
static const std::map<std::string, SolverTypes> SolverTypes_map = SolverTypes_init_map();

enum class MpcTypes { Carrot, Rail, Weighted };

static std::map<std::string, MpcTypes> MpcTypes_init_map()
{
    std::map<std::string, MpcTypes> m;
    m.clear();
    m.insert({"Carrot", MpcTypes::Carrot});
    m.insert({"Rail", MpcTypes::Rail});
    m.insert({"Weighted", MpcTypes::Weighted});
    return m;
}
static const std::map<std::string, MpcTypes> MpcTypes_map = MpcTypes_init_map();

class MpcAbstract
{
    public:
    MpcAbstract(const std::string& yaml_path);

    virtual void createProblem()                                = 0;
    virtual void updateProblem(const std::size_t& current_time) = 0;

    const boost::shared_ptr<pinocchio::Model>&                                        get_robot_model() const;
    const boost::shared_ptr<MultiCopterBaseParams>&                                   get_platform_params() const;
    const boost::shared_ptr<crocoddyl::StateMultibody>&                               get_robot_state() const;
    const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase>&                get_actuation() const;
    const boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>&                      get_squash() const;
    const boost::shared_ptr<crocoddyl::ActuationSquashingModel>&                      get_actuation_squash() const;
    const std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract>>& get_dif_models() const;
    const std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>&             get_int_models() const;
    const boost::shared_ptr<crocoddyl::ShootingProblem>&                              get_problem() const;
    const std::string&                                                                get_robot_model_path() const;
    const boost::shared_ptr<crocoddyl::SolverDDP>&                                    get_solver() const;

    const std::size_t& get_dt() const;
    const std::size_t& get_knots() const;
    const std::size_t& get_iters() const;
    const SolverTypes& get_solver_type() const;

    protected:
    boost::shared_ptr<pinocchio::Model>      robot_model_;
    boost::shared_ptr<MultiCopterBaseParams> platform_params_;
    std::string                              robot_model_path_;

    boost::shared_ptr<crocoddyl::StateMultibody>                robot_state_;
    boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
    boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>       squash_;
    boost::shared_ptr<crocoddyl::ActuationSquashingModel>       actuation_squash_;

    std::vector<boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract>> dif_models_;
    std::vector<boost::shared_ptr<crocoddyl::ActionModelAbstract>>             int_models_;
    boost::shared_ptr<crocoddyl::ShootingProblem>                              problem_;

    std::vector<boost::shared_ptr<crocoddyl::CallbackAbstract>> solver_callbacks_;
    boost::shared_ptr<crocoddyl::SolverDDP>                     solver_;

    boost::shared_ptr<CostModelFactory> cost_factory_;
    boost::shared_ptr<ParamsServer>     params_server_;

    struct MpcParams {
        IntegratedActionModelTypes integrator_type;
        SolverTypes                solver_type;
        std::size_t                knots;
        std::size_t                iters;
        std::size_t                dt;
    } params_;

    private:
    void initializeRobotObjects();
    void loadParams();
};
}  // namespace eagle_mpc

#endif