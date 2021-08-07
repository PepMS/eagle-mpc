///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_MPC_CONTROLLERS_WEIGHTED_MPC_HPP_
#define EAGLE_MPC_MPC_CONTROLLERS_WEIGHTED_MPC_HPP_

// #include "pinocchio/algorithm/joint-configuration.hpp"

#include "crocoddyl/core/costs/cost-sum.hpp"
#include "crocoddyl/multibody/costs/state.hpp"

#include "eagle_mpc/trajectory.hpp"
#include "eagle_mpc/mpc-base.hpp"
#include "eagle_mpc/utils/params_server.hpp"

namespace eagle_mpc
{
class WeightedMpc : public MpcAbstract
{
    public:
    WeightedMpc(const boost::shared_ptr<Trajectory>& trajectory,
                const std::size_t                    dt_ref,
                const std::string&                   yaml_path);

    virtual ~WeightedMpc();

    virtual void createProblem() override;
    virtual void updateProblem(const std::size_t& current_time) override;

    const boost::shared_ptr<Trajectory>& get_trajectory() const;
    const std::vector<std::size_t>&      get_t_stages() const;

    private:
    boost::shared_ptr<crocoddyl::CostModelSum> createCosts() const;

    void computeActiveStage(const std::size_t& current_time);
    void computeActiveStage(const std::size_t& current_time, const std::size_t& last_stage);

    void computeWeight(const std::size_t& time);
    void updateContactCosts(const std::size_t& idx);
    void updateFreeCosts(const std::size_t& idx);

    boost::shared_ptr<Trajectory> trajectory_;

    std::vector<std::size_t> t_stages_;

    struct updateVars {
        std::size_t idx_stage;
        std::size_t idx_last_stage;
        std::size_t t_start_stage;
        std::size_t node_time;
        std::string name_stage;
        double      weight;
        double      weight_time;
        double      exp_1;
        double      exp_2;

        boost::shared_ptr<crocoddyl::DifferentialActionModelContactFwdDynamics> dif_contact;
        boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics>    dif_free;
    } update_vars_;

    double alpha_;
    double beta_;
    double state_reg_;
    double control_reg_;
};
}  // namespace eagle_mpc

#endif
