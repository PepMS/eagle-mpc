#ifndef MULTICOPTER_MPC_FACTORY_DIFF_ACTION_HPP_
#define MULTICOPTER_MPC_FACTORY_DIFF_ACTION_HPP_

#include <map>

#include "crocoddyl/core/diff-action-base.hpp"

#include "crocoddyl/multibody/actions/free-fwddyn.hpp"
#include "crocoddyl/multibody/actions/contact-fwddyn.hpp"

#include "multicopter_mpc/stage.hpp"

namespace multicopter_mpc {

class Stage;

struct DifferentialActionModelTypes {
  enum Type {
    DifferentialActionModelFreeFwdDynamics,
    DifferentialActionModelContactFwdDynamics,
    NbDifferentialActionModelTypes
  };

  static std::map<std::string, Type> init_all() {
    std::map<std::string, Type> m;
    m.clear();
    m.insert({"DifferentialActionModelFreeFwdDynamics", DifferentialActionModelFreeFwdDynamics});
    m.insert({"DifferentialActionModelContactFwdDynamics", DifferentialActionModelContactFwdDynamics});
    return m;
  }
  static const std::map<std::string, Type> all;
};

class DifferentialActionModelFactory {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit DifferentialActionModelFactory();
  ~DifferentialActionModelFactory();

  boost::shared_ptr<crocoddyl::DifferentialActionModelAbstract> create(const bool& is_contact, const bool& squash,
                                                                       const boost::shared_ptr<Stage>& stage) const;
};

}  // namespace multicopter_mpc
#endif