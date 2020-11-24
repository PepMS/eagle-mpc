#include <iostream>

#include <Eigen/Dense>

#include "crocoddyl/core/utils/timer.hpp"

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/mpc-main.hpp"

#define SMOOTH(s) for (std::size_t _smooth = 0; _smooth < s; ++_smooth)

#define STDDEV(vec) std::sqrt(((vec - vec.mean())).square().sum() / ((double)vec.size() - 1))
#define AVG(vec) (vec.mean())

void applyRandomNoise(Eigen::VectorXd& vector) {
  Eigen::VectorXd r_vector = Eigen::VectorXd::Random(vector.size());

  vector += r_vector * 0.05;

  Eigen::Quaterniond quat;
  quat.vec() = vector.segment(3, 3);
  quat.w() = vector(6);
  quat.normalize();

  vector.segment(3, 3) = quat.vec();
  vector(6) = quat.w();
}

class Simulator {
 public:
  Simulator(const double& dt) {
    dt_ = dt;

    std::string model_description_path;
    std::string model_yaml_path;
    std::string mission_yaml_path;

    model_description_path = EXAMPLE_ROBOT_DATA_MODEL_DIR "/iris_description/robots/iris_simple.urdf";
    model_yaml_path = MULTICOPTER_MPC_MULTIROTOR_DIR "/iris.yaml";
    mission_yaml_path = MULTICOPTER_MPC_MISSION_DIR "/passthrough.yaml";

    pinocchio::Model model;
    pinocchio::urdf::buildModel(model_description_path, pinocchio::JointModelFreeFlyer(), model);
    model_ = boost::make_shared<pinocchio::Model>(model);

    mc_params_ = boost::make_shared<multicopter_mpc::MultiCopterBaseParams>();
    mc_params_->fill(model_yaml_path);

    mission_ = boost::make_shared<multicopter_mpc::Mission>(model_->nq + model_->nv);
    mission_->fillWaypoints(mission_yaml_path);

    robot_state_ = boost::make_shared<crocoddyl::StateMultibody>(model_);
    actuation_model_ = boost::make_shared<crocoddyl::ActuationModelMultiCopterBase>(
        robot_state_, mc_params_->n_rotors_, mc_params_->tau_f_);

    boost::shared_ptr<crocoddyl::CostModelSum> costs =
        boost::make_shared<crocoddyl::CostModelSum>(robot_state_, actuation_model_->get_nu());
    dif_model_ =
        boost::make_shared<crocoddyl::DifferentialActionModelFreeFwdDynamics>(robot_state_, actuation_model_, costs);
    int_model_ = boost::make_shared<crocoddyl::IntegratedActionModelRK4>(dif_model_, dt_);
    int_data_ = boost::static_pointer_cast<crocoddyl::IntegratedActionDataRK4>(int_model_->createData());

    state_next_ = robot_state_->zero();
  }

  const Eigen::VectorXd& simulateStep(const Eigen::Ref<const Eigen::VectorXd>& state,
                                      const Eigen::Ref<const Eigen::VectorXd>& control) {
    state_trajectory_.push_back(state);
    control_trajectory_.push_back(control);
    int_model_->calc(int_data_, state, control);
    return int_data_->xnext;
  }

 private:
  boost::shared_ptr<pinocchio::Model> model_;

  boost::shared_ptr<crocoddyl::StateMultibody> robot_state_;
  boost::shared_ptr<crocoddyl::ActuationModelMultiCopterBase> actuation_model_;
  boost::shared_ptr<crocoddyl::DifferentialActionModelFreeFwdDynamics> dif_model_;
  boost::shared_ptr<crocoddyl::IntegratedActionModelRK4> int_model_;
  boost::shared_ptr<crocoddyl::IntegratedActionDataRK4> int_data_;

  boost::shared_ptr<multicopter_mpc::MultiCopterBaseParams> mc_params_;
  boost::shared_ptr<multicopter_mpc::Mission> mission_;

  double dt_;
  Eigen::VectorXd state_next_;
  std::vector<Eigen::VectorXd> state_trajectory_;
  std::vector<Eigen::VectorXd> control_trajectory_;
};