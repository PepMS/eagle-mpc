#ifndef MULTICOPTER_MPC_MPC_CONTROLLERS_CARROT_MPC_HPP_
#define MULTICOPTER_MPC_MPC_CONTROLLERS_CARROT_MPC_HPP_

#include "multicopter_mpc/trajectory.hpp"
#include "multicopter_mpc/mpc-base.hpp"
#include "multicopter_mpc/utils/params_server.hpp"

namespace multicopter_mpc {

class CarrotMpc : public MpcAbstract {
 public:
  CarrotMpc(const boost::shared_ptr<Trajectory>& trajectory, const boost::shared_ptr<ParamsServer>& params_server);

  virtual ~CarrotMpc();

  virtual void createProblem() override;

 private:
  boost::shared_ptr<Trajectory> trajectory_;
};
}  // namespace multicopter_mpc

#endif
