///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_MPC_CONTROLLERS_CARROT_MPC_HPP_
#define EAGLE_MPC_MPC_CONTROLLERS_CARROT_MPC_HPP_

// #include "pinocchio/algorithm/joint-configuration.hpp"

#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/core/costs/residual.hpp"
#include "crocoddyl/multibody/residuals/state.hpp"

#include "eagle_mpc/trajectory.hpp"
#include "eagle_mpc/mpc-base.hpp"
#include "eagle_mpc/utils/params_server.hpp"

namespace eagle_mpc
{
class CarrotMpc : public MpcAbstract
{
    public:
    CarrotMpc(const boost::shared_ptr<Trajectory>& trajectory,
              const std::vector<Eigen::VectorXd>&  state_ref,
              const std::size_t                    dt_ref,
              const std::string&                   yaml_path);

    virtual ~CarrotMpc();

    virtual void createProblem() override;
    virtual void updateProblem(const std::size_t& current_time) override;

    const boost::shared_ptr<Trajectory>& get_trajectory() const;
    const std::vector<Eigen::VectorXd>&  get_state_ref() const;

    const std::vector<std::size_t>& get_t_stages() const;
    const std::vector<std::size_t>& get_t_ref() const;

    private:
    boost::shared_ptr<crocoddyl::CostModelSum> createCosts() const;

    void computeActiveStage(const std::size_t& current_time);
    void computeActiveStage(const std::size_t& current_time, const std::size_t& last_stage);
    void computeStateReference(const std::size_t& time);
    void updateContactCosts(const std::size_t& idx);
    void updateFreeCostsTasks(const std::size_t& idx);
    void updateFreeCosts(const std::size_t& idx, const std::size_t& current_time);
    void loadCostParams();

    boost::shared_ptr<Trajectory> trajectory_;

    std::vector<std::size_t> t_stages_;

    std::vector<Eigen::VectorXd> state_ref_;
    std::vector<std::size_t>     t_ref_;

    struct updateVars {
        std::size_t        idx_stage;
        std::size_t        idx_last_stage;
        std::size_t        node_time;
        std::string        name_stage;
        Eigen::VectorXd    state_ref;
        Eigen::Quaterniond quat_hover;
        std::size_t        idx_state;
        double             alpha;

        boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> dif_contact;
        boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>    dif_free;
    } update_vars_;

    double          carrot_weight_;
    double          carrot_tail_weight_;
    Eigen::VectorXd carrot_tail_act_weights_;

    double          control_reg_weight_;
    Eigen::VectorXd control_reg_act_weights_;
    double          state_reg_weight_;
    Eigen::VectorXd state_ref_act_weights_;
    double          state_limits_weight_;
    Eigen::VectorXd state_limits_act_weights_;
    Eigen::VectorXd state_limits_u_bound_;
    Eigen::VectorXd state_limits_l_bound_;
};
}  // namespace eagle_mpc

#endif
