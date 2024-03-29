///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, IRI: CSIC-UPC
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef EAGLE_MPC_FACTORY_CONTACTS_HPP_
#define EAGLE_MPC_FACTORY_CONTACTS_HPP_

#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

#include "crocoddyl/multibody/contact-base.hpp"
#include "crocoddyl/multibody/contacts/contact-3d.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"

#include "eagle_mpc/stage.hpp"
#include "eagle_mpc/utils/params_server.hpp"
#include "eagle_mpc/factory/activation.hpp"

namespace eagle_mpc
{
enum class ContactModelTypes { ContactModel2D, ContactModel3D, ContactModel6D, NbContactModelTypes };

static std::map<std::string, ContactModelTypes> ContactModelTypes_init_map()
{
    std::map<std::string, ContactModelTypes> m;
    m.clear();
    m.insert({"ContactModel2D", ContactModelTypes::ContactModel2D});
    m.insert({"ContactModel3D", ContactModelTypes::ContactModel3D});
    m.insert({"ContactModel6D", ContactModelTypes::ContactModel6D});
    return m;
}
static const std::map<std::string, ContactModelTypes> ContactModelTypes_map = ContactModelTypes_init_map();

class ContactModelFactory
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit ContactModelFactory();
    ~ContactModelFactory();

    boost::shared_ptr<crocoddyl::ContactModelAbstract> create(const std::string&                     path_to_contact,
                                                              const boost::shared_ptr<ParamsServer>& server,
                                                              const boost::shared_ptr<Stage>&        stage,
                                                              ContactModelTypes& contact_type) const;
};

}  // namespace eagle_mpc

#endif