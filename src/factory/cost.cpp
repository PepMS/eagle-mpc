#include "multicopter_mpc/factory/cost.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

CostModelFactory::CostModelFactory() { activation_factory_ = boost::make_shared<ActivationModelFactory>(); }
CostModelFactory::~CostModelFactory() {}

boost::shared_ptr<crocoddyl::CostModelAbstract> CostModelFactory::create(
    const std::string& path_to_cost, const boost::shared_ptr<ParamsServer>& server,
    const boost::shared_ptr<crocoddyl::StateMultibody>& state, const std::size_t& nu,
    CostModelTypes& cost_type) const {
  boost::shared_ptr<crocoddyl::CostModelAbstract> cost;
  boost::shared_ptr<crocoddyl::ActivationModelAbstract> activation;
  Eigen::VectorXd reference;

  try {
    cost_type = CostModelTypes_map.at(server->getParam<std::string>(path_to_cost + "type"));
  } catch (const std::exception& e) {
    throw std::runtime_error("Cost " + server->getParam<std::string>(path_to_cost + "type") +
                             " not found. Please make sure the specified cost exists.");
  }

  switch (cost_type) {
    case CostModelTypes::CostModelState: {
      activation = activation_factory_->create(path_to_cost, server, state->get_ndx());

      try {
        reference = converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "reference"));
      } catch (const std::exception& e) {
        MMPC_WARN << e.what() << " Set to the zero state vector";
        reference = state->zero();
      }
      if (reference.size() != state->get_nx()) {
        throw std::runtime_error("State reference vector @" + path_to_cost + "reference has dimension " +
                                 std::to_string(reference.size()) + ". Should be " + std::to_string(state->get_nx()));
      }

      cost = boost::make_shared<crocoddyl::CostModelState>(state, activation, reference, nu);
      // cost_type = CostModelTypes::CostModelState;
    } break;
    case CostModelTypes::CostModelControl: {
      activation = activation_factory_->create(path_to_cost, server, nu);

      try {
        reference = converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "reference"));
      } catch (const std::exception& e) {
        MMPC_WARN << e.what() << " Set to the zero control vector";
        reference = Eigen::VectorXd::Zero(nu);
      }
      if (reference.size() != nu) {
        throw std::runtime_error("Control reference vector @" + path_to_cost + "reference has dimension " +
                                 std::to_string(reference.size()) + ". Should be " + std::to_string(nu));
      }

      cost = boost::make_shared<crocoddyl::CostModelControl>(state, activation, reference);
    } break;
    case CostModelTypes::CostModelFramePlacement: {
      activation = activation_factory_->create(path_to_cost, server, 6);

      Eigen::Vector3d position =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "position"));
      Eigen::Vector4d orientation =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "orientation"));

      std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
      std::size_t link_id = state->get_pinocchio()->getFrameId(link_name);
      if (link_id == state->get_pinocchio()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }

      Eigen::Quaterniond quat(orientation);
      quat.normalize();
      pinocchio::SE3 m_ref(quat.toRotationMatrix(), position);

      crocoddyl::FramePlacement frame(state->get_pinocchio()->getFrameId(link_name), m_ref);
      cost = boost::make_shared<crocoddyl::CostModelFramePlacement>(state, activation, frame, nu);
    } break;
    case CostModelTypes::CostModelFrameRotation: {
      activation = activation_factory_->create(path_to_cost, server, 3);

      Eigen::Vector4d orientation =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "orientation"));

      std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
      std::size_t link_id = state->get_pinocchio()->getFrameId(link_name);
      if (link_id == state->get_pinocchio()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }

      Eigen::Quaterniond quat(orientation);
      quat.normalize();
      crocoddyl::FrameRotation frame(state->get_pinocchio()->getFrameId(link_name), quat.toRotationMatrix());
      cost = boost::make_shared<crocoddyl::CostModelFrameRotation>(state, activation, frame, nu);
    } break;
    case CostModelTypes::CostModelFrameVelocity: {
      activation = activation_factory_->create(path_to_cost, server, 6);

      Eigen::Vector3d linear =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "linear"));
      Eigen::Vector3d angular =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "angular"));
      std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
      std::size_t link_id = state->get_pinocchio()->getFrameId(link_name);
      if (link_id == state->get_pinocchio()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }

      pinocchio::Motion motion_ref(linear, angular);

      crocoddyl::FrameMotion frame(link_id, motion_ref);
      cost = boost::make_shared<crocoddyl::CostModelFrameVelocity>(state, activation, frame, nu);
    } break;
    case CostModelTypes::CostModelFrameTranslation: {
      activation = activation_factory_->create(path_to_cost, server, 3);

      Eigen::Vector3d position =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "position"));
      std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
      std::size_t link_id = state->get_pinocchio()->getFrameId(link_name);
      if (link_id == state->get_pinocchio()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }

      crocoddyl::FrameTranslation frame(link_id, position);
      cost = boost::make_shared<crocoddyl::CostModelFrameTranslation>(state, activation, frame, nu);
    } break;
    case CostModelTypes::CostModelContactFrictionCone: {
      Eigen::Vector3d n_surf =
          converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "n_surf"));
      double mu = server->getParam<double>(path_to_cost + "mu");

      crocoddyl::FrictionCone friction_cone(n_surf, mu, 4, false);
      std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
      std::size_t link_id = state->get_pinocchio()->getFrameId(link_name);
      if (link_id == state->get_pinocchio()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }
      crocoddyl::FrameFrictionCone frame(link_id, friction_cone);

      // In a constrained solver this might not be useful
      crocoddyl::ActivationBounds bounds(friction_cone.get_lb(), friction_cone.get_ub());
      activation = boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);

      cost = boost::make_shared<crocoddyl::CostModelContactFrictionCone>(state, activation, frame, nu);
    } break;
  }
  return cost;
}

}  // namespace multicopter_mpc