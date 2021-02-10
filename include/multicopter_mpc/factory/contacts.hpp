///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (C) 2018-2020, LAAS-CNRS, University of Edinburgh, IRI: CSIC-UPC
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef MULTICOPTER_MPC_FACTORY_CONTACTS_HPP_
#define MULTICOPTER_MPC_FACTORY_CONTACTS_HPP_

#include <vector>
#include <string>
#include <map>

#include <Eigen/Dense>

#include "crocoddyl/multibody/contact-base.hpp"
#include "crocoddyl/multibody/contacts/contact-3d.hpp"
#include "crocoddyl/multibody/contacts/contact-6d.hpp"

#include "multicopter_mpc/stage.hpp"
#include "multicopter_mpc/utils/params_server.hpp"
#include "multicopter_mpc/factory/activation.hpp"

namespace multicopter_mpc {
enum class ContactModelTypes { ContactModel2D, ContactModel3D, ContactModel6D, NbContactModelTypes };

static std::map<std::string, ContactModelTypes> ContactModelTypes_init_map() {
  std::map<std::string, ContactModelTypes> m;
  m.clear();
  m.insert({"ContactModel2D", ContactModelTypes::ContactModel2D});
  m.insert({"ContactModel3D", ContactModelTypes::ContactModel3D});
  m.insert({"ContactModel6D", ContactModelTypes::ContactModel6D});
  return m;
}
static const std::map<std::string, ContactModelTypes> ContactModelTypes_map = ContactModelTypes_init_map();

class ContactModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit ContactModelFactory();
  ~ContactModelFactory();

  boost::shared_ptr<crocoddyl::ContactModelAbstract> create(const std::string& path_to_contact,
                                                            const ParamsServer& server,
                                                            const boost::shared_ptr<Stage>& stage,
                                                            ContactModelTypes& contact_type) const;
};

}  // namespace multicopter_mpc

#endif