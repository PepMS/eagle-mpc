#include "multicopter_mpc/factory/contacts.hpp"

#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {

const std::map<std::string, ContactModelTypes::Type> ContactModelTypes::all = ContactModelTypes::init_all();

ContactModelFactory::ContactModelFactory() {}
ContactModelFactory::~ContactModelFactory() {}

boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelFactory::create(
    const std::string& path_to_contact, const ParamsServer& server, const boost::shared_ptr<Stage>& stage) const {
  boost::shared_ptr<crocoddyl::ContactModelAbstract> contact;

  switch (ContactModelTypes::all.at(server.getParam<std::string>(path_to_contact + "type"))) {
    case ContactModelTypes::ContactModel3D: {
      Eigen::Vector3d position =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_contact + "position"));
      std::string link_name = server.getParam<std::string>(path_to_contact + "link_name");
      std::size_t link_id = stage->get_trajectory()->get_robot_model()->getFrameId(link_name);
      if (link_id == stage->get_trajectory()->get_robot_model()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }
      Eigen::Vector2d gains;
      try {
        gains = converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_contact + "gains"));
      } catch (const std::exception& e) {
        MMPC_WARN << e.what() << " Set to the zero gains vector";
        gains = Eigen::Vector2d::Zero();
      }

      crocoddyl::FrameTranslation frame(link_id, position);
      contact =
          boost::make_shared<crocoddyl::ContactModel3D>(stage->get_trajectory()->get_robot_state(), frame,
                                                        stage->get_trajectory()->get_actuation()->get_nu(), gains);
    } break;
    case ContactModelTypes::ContactModel6D: {
      Eigen::Vector3d position =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_contact + "position"));
      Eigen::Vector4d orientation =
          converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_contact + "orientation"));

      std::string link_name = server.getParam<std::string>(path_to_contact + "link_name");
      std::size_t link_id = stage->get_trajectory()->get_robot_model()->getFrameId(link_name);
      if (link_id == stage->get_trajectory()->get_robot_model()->frames.size()) {
        throw std::runtime_error("Link " + link_name + "does no exists");
      }

      Eigen::Quaterniond quat(orientation);
      quat.normalize();
      pinocchio::SE3 m_ref(quat.toRotationMatrix(), position);

      crocoddyl::FramePlacement frame(stage->get_trajectory()->get_robot_model()->getFrameId(link_name), m_ref);

      Eigen::Vector2d gains;
      try {
        gains = converter<Eigen::VectorXd>::convert(server.getParam<std::string>(path_to_contact + "gains"));
      } catch (const std::exception& e) {
        MMPC_WARN << e.what() << " Set to the zero gains vector";
        gains = Eigen::Vector2d::Zero();
      }

      contact =
          boost::make_shared<crocoddyl::ContactModel6D>(stage->get_trajectory()->get_robot_state(), frame,
                                                        stage->get_trajectory()->get_actuation()->get_nu(), gains);
    } break;
    default:
      throw std::runtime_error("Cost " + server.getParam<std::string>(path_to_contact + "type") +
                               "not found. Please make sure the specified cost exists.");
      break;
  }
  return contact;
}

}  // namespace multicopter_mpc