///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc/factory/cost.hpp"

#include "eagle_mpc/utils/log.hpp"

namespace eagle_mpc
{
CostModelFactory::CostModelFactory() { activation_factory_ = boost::make_shared<ActivationModelFactory>(); }
CostModelFactory::~CostModelFactory() {}

boost::shared_ptr<crocoddyl::CostModelResidual> CostModelFactory::create(
    const std::string&                                  path_to_cost,
    const boost::shared_ptr<ParamsServer>&              server,
    const boost::shared_ptr<crocoddyl::StateMultibody>& state,
    const std::size_t&                                  nu,
    CostModelTypes&                                     cost_type) const
{
    boost::shared_ptr<crocoddyl::CostModelResidual>       cost;
    boost::shared_ptr<crocoddyl::ResidualModelAbstract>   residual;
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
                reference =
                    converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "reference"));
            } catch (const std::exception& e) {
                EMPC_WARN << e.what() << " Set to the zero state vector";
                reference = state->zero();
            }
            if (reference.size() != state->get_nx()) {
                throw std::runtime_error("State reference vector @" + path_to_cost + "reference has dimension " +
                                         std::to_string(reference.size()) + ". Should be " +
                                         std::to_string(state->get_nx()));
            }

            residual = boost::make_shared<crocoddyl::ResidualModelState>(state, reference, nu);
            cost     = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
        case CostModelTypes::CostModelControl: {
            activation = activation_factory_->create(path_to_cost, server, nu);

            try {
                reference =
                    converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "reference"));
            } catch (const std::exception& e) {
                EMPC_WARN << e.what() << " Set to the zero control vector";
                reference = Eigen::VectorXd::Zero(nu);
            }
            if (reference.size() != nu) {
                throw std::runtime_error("Control reference vector @" + path_to_cost + "reference has dimension " +
                                         std::to_string(reference.size()) + ". Should be " + std::to_string(nu));
            }

            residual = boost::make_shared<crocoddyl::ResidualModelControl>(state, reference);
            cost     = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
        case CostModelTypes::CostModelFramePlacement: {
            activation = activation_factory_->create(path_to_cost, server, 6);

            Eigen::Vector3d position =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "position"));
            Eigen::Vector4d orientation =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "orientation"));

            std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
            std::size_t link_id   = state->get_pinocchio()->getFrameId(link_name);
            if (link_id == state->get_pinocchio()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }

            Eigen::Quaterniond quat(orientation);
            quat.normalize();

            pinocchio::SE3 m_ref(quat.toRotationMatrix(), position);

            residual = boost::make_shared<crocoddyl::ResidualModelFramePlacement>(state, link_id, m_ref, nu);
            cost     = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
        case CostModelTypes::CostModelFrameRotation: {
            activation = activation_factory_->create(path_to_cost, server, 3);

            Eigen::Vector4d orientation =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "orientation"));

            std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
            std::size_t link_id   = state->get_pinocchio()->getFrameId(link_name);
            if (link_id == state->get_pinocchio()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }

            Eigen::Quaterniond quat(orientation);
            quat.normalize();

            residual =
                boost::make_shared<crocoddyl::ResidualModelFrameRotation>(state, link_id, quat.toRotationMatrix(), nu);
            cost = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
        case CostModelTypes::CostModelFrameVelocity: {
            activation = activation_factory_->create(path_to_cost, server, 6);

            Eigen::Vector3d linear =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "linear"));
            Eigen::Vector3d angular =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "angular"));
            std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
            std::size_t link_id   = state->get_pinocchio()->getFrameId(link_name);
            if (link_id == state->get_pinocchio()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }

            pinocchio::Motion motion_ref(linear, angular);

            residual = boost::make_shared<crocoddyl::ResidualModelFrameVelocity>(state, link_id, motion_ref,
                                                                                 pinocchio::ReferenceFrame::LOCAL, nu);
            cost     = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
        case CostModelTypes::CostModelFrameTranslation: {
            activation = activation_factory_->create(path_to_cost, server, 3);

            Eigen::Vector3d position =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "position"));
            std::string link_name = server->getParam<std::string>(path_to_cost + "link_name");
            std::size_t link_id   = state->get_pinocchio()->getFrameId(link_name);
            if (link_id == state->get_pinocchio()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }

            residual = boost::make_shared<crocoddyl::ResidualModelFrameTranslation>(state, link_id, position, nu);
            cost     = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
        case CostModelTypes::CostModelContactFrictionCone: {
            Eigen::Vector3d n_surf =
                converter<Eigen::VectorXd>::convert(server->getParam<std::string>(path_to_cost + "n_surf"));
            double mu = server->getParam<double>(path_to_cost + "mu");

            crocoddyl::FrictionCone friction_cone(n_surf, mu, 4, false);
            std::string             link_name = server->getParam<std::string>(path_to_cost + "link_name");
            std::size_t             link_id   = state->get_pinocchio()->getFrameId(link_name);
            if (link_id == state->get_pinocchio()->frames.size()) {
                throw std::runtime_error("Link " + link_name + "does no exists");
            }

            // In a constrained solver this might not be useful
            crocoddyl::ActivationBounds bounds(friction_cone.get_lb(), friction_cone.get_ub());
            activation = boost::make_shared<crocoddyl::ActivationModelQuadraticBarrier>(bounds);

            residual =
                boost::make_shared<crocoddyl::ResidualModelContactFrictionCone>(state, link_id, friction_cone, nu);
            cost = boost::make_shared<crocoddyl::CostModelResidual>(state, activation, residual);
        } break;
    }
    return cost;
}

}  // namespace eagle_mpc