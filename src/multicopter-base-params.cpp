#include "multicopter_mpc/multicopter-base-params.hpp"

namespace multicopter_mpc {

MultiCopterBaseParams::MultiCopterBaseParams() {}
MultiCopterBaseParams::MultiCopterBaseParams(double cf, double cm, Eigen::MatrixXd tau_f, double max_th, double min_th)
    : cf_(cf), cm_(cm), n_rotors_(tau_f.cols()), tau_f_(tau_f), max_thrust_(max_th), min_thrust_(min_th) {}

MultiCopterBaseParams::MultiCopterBaseParams(double cf, double cm, Eigen::MatrixXd tau_f, double max_th, double min_th,
                                             Eigen::VectorXd max_torque, Eigen::VectorXd min_torque)
    : cf_(cf),
      cm_(cm),
      n_rotors_(tau_f.cols()),
      tau_f_(tau_f),
      max_thrust_(max_th),
      min_thrust_(min_th),
      max_torque_(max_torque),
      min_torque_(min_torque) {}
MultiCopterBaseParams::~MultiCopterBaseParams() {}

void MultiCopterBaseParams::fill(const yaml_parser::ParamsServer& server) {
  cf_ = server.getParam<double>("multirotor/cf");
  cm_ = server.getParam<double>("multirotor/cm");
  max_thrust_ = server.getParam<double>("multirotor/max_thrust");
  min_thrust_ = server.getParam<double>("multirotor/min_thrust");

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
}
}  // namespace multicopter_mpc