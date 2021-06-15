#include "eagle_mpc/multicopter-base-params.hpp"

#include "eagle_mpc/utils/log.hpp"

namespace eagle_mpc {

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
MultiCopterBaseParams::~MultiCopterBaseParams() {}

void MultiCopterBaseParams::autoSetup(const std::string& path_to_platform,
                                      const boost::shared_ptr<ParamsServer>& server) {
  try {
    cf_ = server->getParam<double>(path_to_platform + "cf");
    cm_ = server->getParam<double>(path_to_platform + "cm");
    max_thrust_ = server->getParam<double>(path_to_platform + "max_thrust");
    min_thrust_ = server->getParam<double>(path_to_platform + "min_thrust");
    // This parameter may be removed after refactor
    base_link_name_ = server->getParam<std::string>(path_to_platform + "base_link_name");
    n_rotors_ = server->getParam<int>(path_to_platform + "n_rotors");
    MMPC_INFO << "Number of rotors: " << n_rotors_;
    std::vector<std::string> rotors = server->getParam<std::vector<std::string>>(path_to_platform + "rotors");
    if (n_rotors_ != rotors.size()) {
      throw std::runtime_error("'n_rotors' field and the number of rotor poses specified must be the same.");
    }

    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
    for (int i = 0; i < n_rotors_; ++i) {
      std::map<std::string, Eigen::VectorXd> rotor =
          converter<std::map<std::string, Eigen::VectorXd>>::convert(rotors[i]);

      translation = rotor["translation"];
      orientation = Eigen::Quaterniond(Eigen::Vector4d(rotor["orientation"]));
      orientation.normalize();
      pinocchio::SE3 rotor_pose(orientation.toRotationMatrix(), translation);
      rotors_pose_.push_back(rotor_pose);

      int spin_dir = int(rotor["spin_direction"](0));
      rotors_spin_dir_.push_back(spin_dir);
    }
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
  }

  Eigen::Vector3d e3;
  Eigen::Vector3d thrust_w = Eigen::Vector3d::Zero();
  Eigen::Vector3d torque_yaw = Eigen::Vector3d::Zero();
  e3 << 0, 0, 1;
  tau_f_ = Eigen::MatrixXd::Zero(6, n_rotors_);
  for (std::size_t i = 0; i < rotors_pose_.size(); ++i) {
    thrust_w = rotors_pose_[i].rotation() * e3;
    tau_f_.block<3, 1>(0, i) = thrust_w;

    torque_yaw = rotors_spin_dir_[i] * cm_ / cf_ * thrust_w;
    tau_f_.block<3, 1>(3, i) = rotors_pose_[i].translation().cross(thrust_w) + torque_yaw;
  }
}

void MultiCopterBaseParams::autoSetup(const std::string& path_to_platform,
                                      const boost::shared_ptr<ParamsServer>& server,
                                      const boost::shared_ptr<pinocchio::Model>& robot_model) {
  autoSetup(path_to_platform, server);
  setControlLimits(robot_model);
}

void MultiCopterBaseParams::setControlLimits(const boost::shared_ptr<pinocchio::Model>& robot_model) {
  std::size_t n_arm_joints = robot_model->nq - 7;

  u_lb = Eigen::VectorXd::Zero(n_arm_joints + n_rotors_);
  u_ub = u_lb;

  u_lb.head(n_rotors_) = Eigen::VectorXd::Ones(n_rotors_) * min_thrust_;
  u_ub.head(n_rotors_) = Eigen::VectorXd::Ones(n_rotors_) * max_thrust_;

  u_lb.tail(n_arm_joints) = -robot_model->effortLimit.tail(n_arm_joints);
  u_ub.tail(n_arm_joints) = robot_model->effortLimit.tail(n_arm_joints);
}

}  // namespace eagle_mpc