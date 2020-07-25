#ifndef MULTICOPTER_MPC_MPC_BASE_HPP_
#define MULTICOPTER_MPC_MPC_BASE_HPP_

#include <map>

#include <boost/shared_ptr.hpp>

#include "multicopter_mpc/ocp/ocp-base.hpp"
#include "multicopter_mpc/ocp/trajectory-generator.hpp"
#include "multicopter_mpc/mission.hpp"

namespace multicopter_mpc {

class MpcAbstract : public OcpAbstract {
 public:
  MpcAbstract(const boost::shared_ptr<pinocchio::Model>& model,
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const double& dt,
              const boost::shared_ptr<Mission>& mission, const std::size_t& n_knots);

  ~MpcAbstract();

  virtual void updateProblem(const std::size_t idx_trajectory) = 0;

  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;
  const boost::shared_ptr<TrajectoryGenerator> getTrajectoryGenerator() const;
  const boost::shared_ptr<Mission> getMission() const;

 protected:
  virtual void initializeTrajectoryGenerator(const SolverTypes::Type& solver_type) = 0;

  boost::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  boost::shared_ptr<Mission> mission_;
};

class FactoryMpc {
 public:
  FactoryMpc();
  ~FactoryMpc();

  using createMethod = boost::shared_ptr<MpcAbstract> (*)(const boost::shared_ptr<pinocchio::Model>&,
                                                          const boost::shared_ptr<MultiCopterBaseParams>&,
                                                          const double&, const boost::shared_ptr<Mission>&,
                                                          const std::size_t&);

  static bool registerMpcController(const std::string& mpc_name, createMethod create_method);
  static boost::shared_ptr<MpcAbstract> createMpcController(const std::string& mpc_name,
                                                            const boost::shared_ptr<pinocchio::Model>& model,
                                                            const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                                            const double& dt, const boost::shared_ptr<Mission>& mission,
                                                            const std::size_t& n_knots);

 private:
  static std::map<std::string, createMethod> s_methods_;
};

}  // namespace multicopter_mpc

#endif