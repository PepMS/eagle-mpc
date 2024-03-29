///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc/factory/contacts.hpp"

#include "eagle_mpc/utils/log.hpp"

namespace eagle_mpc
{
ContactModelFactory::ContactModelFactory() {}
ContactModelFactory::~ContactModelFactory() {}

boost::shared_ptr<crocoddyl::ContactModelAbstract> ContactModelFactory::create(
    const std::string&                     path_to_contact,
    const boost::shared_ptr<ParamsServer>& server,
    const boost::shared_ptr<Stage>&        stage,
    ContactModelTypes&                     contact_type) const
{
    boost::shared_ptr<crocoddyl::ContactModelAbstract> contact;

    try {
        contact_type = ContactModelTypes_map.at(server->getParam<std::string>(path_to_contact + "type"));
    } catch (const std::exception& e) {
        throw std::runtime_error("Contact " + server->getParam<std::string>(path_to_contact + "type") +
                                 "not found. Please make sure the specified contact exists.");
    }

    switch (contact_type) {
        case ContactModelTypes::ContactModel3D: {
            Eigen::Vector3d position =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_contact + "position"));
            std::string link_name = server->getParam<std::string>(path_to_contact + "link_name");
            std::size_t link_id   = stage->get_trajectory()->get_robot_model()->getFrameId(link_name);
            if (link_id == stage->get_trajectory()->get_robot_model()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }
            Eigen::Vector2d gains;
            try {
                gains = converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_contact + "gains"));
            } catch (const std::exception& e) {
                EMPC_DEBUG(e.what(), " Set to the zero gains vector");
                gains = Eigen::Vector2d::Zero();
            }

            contact = boost::make_shared<crocoddyl::ContactModel3D>(
                stage->get_trajectory()->get_robot_state(), link_id, position,
                stage->get_trajectory()->get_actuation()->get_nu(), gains);
        } break;
        case ContactModelTypes::ContactModel6D: {
            Eigen::Vector3d position =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_contact + "position"));
            Eigen::Vector4d orientation =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_contact + "orientation"));

            std::string link_name = server->getParam<std::string>(path_to_contact + "link_name");
            std::size_t link_id   = stage->get_trajectory()->get_robot_model()->getFrameId(link_name);
            if (link_id == stage->get_trajectory()->get_robot_model()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }

            Eigen::Quaterniond quat(orientation);
            quat.normalize();
            pinocchio::SE3 m_ref(quat.toRotationMatrix(), position);

            Eigen::Vector2d gains;
            try {
                gains = converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_contact + "gains"));
            } catch (const std::exception& e) {
                EMPC_DEBUG(e.what(), "  Set to the zero gains vector");
                gains = Eigen::Vector2d::Zero();
            }

            contact = boost::make_shared<crocoddyl::ContactModel6D>(
                stage->get_trajectory()->get_robot_state(), link_id, m_ref,
                stage->get_trajectory()->get_actuation()->get_nu(), gains);
        } break;
    }
    return contact;
}

}  // namespace eagle_mpc