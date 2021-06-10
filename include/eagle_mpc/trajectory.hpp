#ifndef EAGLE_MPC_TRAJECTORY_HPP_
#define EAGLE_MPC_TRAJECTORY_HPP_

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "boost/enable_shared_from_this.hpp"

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"

#include "crocoddyl/core/actuation/squashing/smooth-sat.hpp"
#include "crocoddyl/core/actuation/actuation-squashing.hpp"
#include "crocoddyl/core/optctrl/shooting.hpp"
#include "crocoddyl/core/diff-action-base.hpp"

#include "crocoddyl/multibody/actuations/multicopter-base.hpp"
#include "crocoddyl/multibody/states/multibody.hpp"

#include "eagle_mpc/multicopter-base-params.hpp"
#include "eagle_mpc/stage.hpp"
#include "eagle_mpc/utils/params_server.hpp"
#include "eagle_mpc/utils/parser_yaml.hpp"
#include "eagle_mpc/factory/diff-action.hpp"
#include "eagle_mpc/factory/int-action.hpp"

namespace eagle_mpc {
class Stage;
class DifferentialActionModelFactory;
class IntegratedActionModelFactory;

class Trajectory : public boost::enable_shared_from_this<Trajectory> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ~Trajectory();
  static boost::shared_ptr<Trajectory> create();

  void autoSetup(const std::string& yaml_path);
  boost::shared_ptr<crocoddyl::ShootingProblem> createProblem(const std::size_t& dt, const bool& squash,
                                                              const std::string& integration_method) const;

  void removeStage(const std::size_t& idx_stage);

  void set_initial_state(const Eigen::VectorXd& initial_state);

  const std::vector<boost::shared_ptr<Stage>>& get_stages() const;
  const boost::shared_ptr<pinocchio::Model>& get_robot_model() const;
  const boost::shared_ptr<MultiCopterBaseParams>& get_platform_params() const;
  const boost::shared_ptr<crocoddyl::StateMultibody>& get_robot_state() const;
  const boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase>& get_actuation() const;
  const boost::shared_ptr<crocoddyl::SquashingModelSmoothSat>& get_squash() const;
  const boost::shared_ptr<crocoddyl::ActuationSquashingModel>& get_actuation_squash() const;
  const std::string& get_robot_model_path() const;
  const Eigen::VectorXd& get_initial_state() const;
  const boost::shared_ptr<ParamsServer>& get_params_server() const;
  const bool& get_has_contact() const;
  const std::size_t& get_duration() const;

 private:
  Trajectory();
  std::vector<boost::shared_ptr<Stage>> stages_;

  boost::shared_ptr<pinocchio::Model> robot_model_;
  boost::shared_ptr<MultiCopterBaseParams> platform_params_;
  std::string robot_model_path_;

  boost::shared_ptr<crocoddyl::StateMultibody> robot_state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_;
  boost::shared_ptr<crocoddyl::SquashingModelSmoothSat> squash_;
  boost::shared_ptr<crocoddyl::ActuationSquashingModel> actuation_squash_;

  Eigen::VectorXd initial_state_;

  bool has_contact_;
  std::size_t duration_;

  boost::shared_ptr<ParamsServer> params_server_;

  boost::shared_ptr<DifferentialActionModelFactory> dam_factory_;
  boost::shared_ptr<IntegratedActionModelFactory> iam_factory_;
};
}  // namespace eagle_mpc

#endif