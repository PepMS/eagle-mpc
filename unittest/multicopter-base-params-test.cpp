#define BOOST_TEST_MODULE multicopter_base_params_test test

#include <boost/test/unit_test.hpp>
#include <algorithm>

#include "multicopter_mpc/path.h"
#include "multicopter_mpc/multicopter-base-params.hpp"

#include "yaml_parser/parser_yaml.h"
#include "yaml_parser/params_server.hpp"

BOOST_AUTO_TEST_SUITE(multicopter_mpc_mcparams_test)

BOOST_AUTO_TEST_CASE(constructors_test, *boost::unit_test::tolerance(1e-7)) {
  double cf = 6.6e-5;
  double cm = 1.0e-6;
  double max_thrust = 5.0;
  double min_thrust = 0.1;
  double d_cog = 0.1525;
  std::string base_link_name = "base_link";
  Eigen::MatrixXd tau_f(6, 4);
  tau_f << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, d_cog, 0.0, -d_cog, -d_cog, 0.0, d_cog,
      0.0, -cm / cf, cm / cf, -cm / cf, cm / cf;

  multicopter_mpc::MultiCopterBaseParams m00(cf, cm, tau_f, max_thrust, min_thrust, base_link_name);

  BOOST_CHECK(cf == m00.cf_);
  BOOST_CHECK(cm == m00.cm_);
  BOOST_CHECK(max_thrust == m00.max_thrust_);
  BOOST_CHECK(min_thrust == m00.min_thrust_);
  BOOST_CHECK(tau_f == m00.tau_f_);
  BOOST_CHECK(base_link_name == m00.base_link_name_);
}

BOOST_AUTO_TEST_CASE(fill_test, *boost::unit_test::tolerance(1e-7)) {
  double cf = 6.6e-5;
  double cm = 1.0e-6;
  double max_thrust = 5.0;
  double min_thrust = 0.1;
  double d_cog = 0.1525;
  std::string base_link_name = "base_link";
  Eigen::MatrixXd tau_f(6, 4);
  tau_f << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0, 0.0, d_cog, 0.0, -d_cog, -d_cog, 0.0, d_cog,
      0.0, -cm / cf, cm / cf, -cm / cf, cm / cf;

  std::string mc_params_yaml_path =
      std::string(MULTICOPTER_MPC_ROOT_DIR) + "/unittest/config/multirotor/hector-test.yaml";
  yaml_parser::ParserYAML yaml_mc_params(mc_params_yaml_path, "", true);
  yaml_parser::ParamsServer server_mc_params(yaml_mc_params.getParams());

  multicopter_mpc::MultiCopterBaseParams m00;
  m00.fill(server_mc_params);

  BOOST_CHECK(cf == m00.cf_);
  BOOST_CHECK(cm == m00.cm_);
  BOOST_CHECK(max_thrust == m00.max_thrust_);
  BOOST_CHECK(min_thrust == m00.min_thrust_);
  BOOST_CHECK(tau_f == m00.tau_f_);
  BOOST_CHECK(base_link_name == m00.base_link_name_);
}

BOOST_AUTO_TEST_SUITE_END()