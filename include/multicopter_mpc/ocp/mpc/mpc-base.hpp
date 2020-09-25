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
              const boost::shared_ptr<MultiCopterBaseParams>& mc_params, const boost::shared_ptr<Mission>& mission);

  virtual ~MpcAbstract();

  virtual void updateProblem(const std::size_t idx_trajectory) = 0;

  virtual void loadParameters(const std::string& yaml_path) override;

  virtual void setNumberKnots(const std::size_t& n_knots);

  const Eigen::VectorXd& getControls(const std::size_t& idx = 0) const;
  const Eigen::VectorXd& getFeedForwardGains(const std::size_t& idx = 0) const;
  const Eigen::MatrixXd& getFeedBackGains(const std::size_t& idx = 0) const;
  const boost::shared_ptr<TrajectoryGenerator> getTrajectoryGenerator() const;
  const boost::shared_ptr<Mission> getMission() const;

  using OcpAbstract::createProblem;

  virtual void printInfo();
 protected:
  void initializeTrajectoryGenerator();
  virtual void generateMission();

  boost::shared_ptr<TrajectoryGenerator> trajectory_generator_;
  boost::shared_ptr<Mission> mission_;

  struct TrajectoryGeneratorSpecs {
    std::string yaml_path;
    std::string initial_guess;
    SolverTypes::Type solver;
    IntegratorTypes::Type integrator;
  } trajectory_generator_specs_;
};

class FactoryMpc {
 public:
  FactoryMpc();
  ~FactoryMpc();

  static FactoryMpc& get();

  using createMethod = boost::shared_ptr<MpcAbstract> (*)(const boost::shared_ptr<pinocchio::Model>&,
                                                          const boost::shared_ptr<MultiCopterBaseParams>&,
                                                          const boost::shared_ptr<Mission>&);

  bool registerMpcController(const std::string& mpc_name, createMethod create_method);
  boost::shared_ptr<MpcAbstract> createMpcController(const std::string& mpc_name,
                                                     const boost::shared_ptr<pinocchio::Model>& model,
                                                     const boost::shared_ptr<MultiCopterBaseParams>& mc_params,
                                                     const boost::shared_ptr<Mission>& mission);

 private:
  std::map<std::string, createMethod> s_methods_;
};

}  // namespace multicopter_mpc

#endif