#include "multicopter_mpc/mpc-controllers/carrot-mpc.hpp"

namespace multicopter_mpc {

CarrotMpc::CarrotMpc(const boost::shared_ptr<Trajectory>& trajectory,
                     const boost::shared_ptr<ParamsServer>& params_server)
    : MpcAbstract(params_server), trajectory_(trajectory) {
  createProblem();
}

CarrotMpc::~CarrotMpc() {}

void CarrotMpc::createProblem() {
  for (auto stage = trajectory_->get_stages().begin(); stage != trajectory_->get_stages().end(); ++stage) {
    std::string path_to_stage = "stages/" + (*stage)->get_name();
    for (auto ctype = (*stage)->get_cost_types().begin(); ctype != (*stage)->get_cost_types().end(); ++ctype) {
      std::string cost_name = ctype->first;
      CostModelTypes cost_type = ctype->second;
      boost::shared_ptr<crocoddyl::CostModelAbstract> cost =
          cost_factory_->create(path_to_stage + "/costs/" + cost_name + "/", trajectory_->get_params_server(),
                                robot_state_, actuation_->get_nu(), cost_type);
      costs_->addCost((*stage)->get_name() + "/" + cost_name, cost,
                      (*stage)->get_costs()->get_costs().at(cost_name)->weight, false);
    }
  }
}
}  // namespace multicopter_mpc
