#include "multicopter_mpc/multicopter-base-params.hpp"

#include "multicopter_mpc/utils/log.hpp"
namespace multicopter_mpc {

MultiCopterBaseParams::MultiCopterBaseParams() {}
MultiCopterBaseParams::MultiCopterBaseParams(const double& cf, const double& cm, const Eigen::MatrixXd& tau_f,
                                             const double& max_th, const double& min_th, const std::string& base_link)
    : cf_(cf),
      cm_(cm),
      n_rotors_(tau_f.cols()),
      tau_f_(tau_f),
      max_thrust_(max_th),
      min_thrust_(min_th),
      base_link_name_(base_link) {}

// MultiCopterBaseParams::MultiCopterBaseParams(double cf, double cm, Eigen::MatrixXd tau_f, double max_th, double
// min_th,
//                                              Eigen::VectorXd max_torque, Eigen::VectorXd min_torque,
//                                              const std::string& base_link)
//     : cf_(cf),
//       cm_(cm),
//       n_rotors_(tau_f.cols()),
//       tau_f_(tau_f),
//       max_thrust_(max_th),
//       min_thrust_(min_th),
//       max_torque_(max_torque),
//       min_torque_(min_torque),
//       base_link_name_(base_link) {}
MultiCopterBaseParams::~MultiCopterBaseParams() {}

void MultiCopterBaseParams::fill(const std::string& yaml_path) {
  yaml_parser::ParserYAML yaml_params(yaml_path, "", true);
  yaml_parser::ParamsServer server(yaml_params.getParams());

  cf_ = server.getParam<double>("multirotor/cf");
  cm_ = server.getParam<double>("multirotor/cm");
  max_thrust_ = server.getParam<double>("multirotor/max_thrust");
  min_thrust_ = server.getParam<double>("multirotor/min_thrust");
  min_thrust_ = server.getParam<double>("multirotor/min_thrust");
  base_link_name_ = server.getParam<std::string>("multirotor/base_link_name");

  std::vector<std::string> rotors = server.getParam<std::vector<std::string>>("multirotor/rotors");

  n_rotors_ = rotors.size();
  Eigen::MatrixXd S = Eigen::MatrixXd::Zero(6, n_rotors_);

  for (int ii = 0; ii < n_rotors_; ++ii) {
    std::map<std::string, double> rotor = yaml_parser::converter<std::map<std::string, double>>::convert(rotors[ii]);

    double x = rotor["x"];
    double y = rotor["y"];
    double z = rotor["z"];

    S(2, ii) = 1.0;            // Thrust
    S(3, ii) = y;              // Mx
    S(4, ii) = -x;             // My
    S(5, ii) = z * cm_ / cf_;  // Mz
  }
  tau_f_ = S;

  try {
    max_torque_ = server.getParam<Eigen::VectorXd>("arm/max_torque");
    min_torque_ = server.getParam<Eigen::VectorXd>("arm/min_torque");
  } catch (const std::exception& e) {
    MMPC_INFO << "Multicopter params: Multirotor detected, no arm.";
  }
}
}  // namespace multicopter_mpc