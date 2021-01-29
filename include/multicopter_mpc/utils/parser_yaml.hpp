#ifndef MULTICOPTER_MPC_UTILS_PARSER_YAML_HPP_
#define MULTICOPTER_MPC_UTILS_PARSER_YAML_HPP_

#include <string>
#include <regex>
#include <cassert>
#include <numeric>

#include "yaml-cpp/yaml.h"

#include "multicopter_mpc/utils/converter.hpp"

namespace multicopter_mpc {

class ParserYaml {
 public:
  ParserYaml(std::string file, std::string path_root = "", const bool& freely_parse = false);
  const std::map<std::string, std::string>& get_params();

 private:
  void parse();
  void parseFreely();
  void parseFirstLevel(std::string file_path);

  YAML::Node loadYaml(std::string file_path);

  std::string generatePath(std::string);

  void walkTreeRecursive(YAML::Node node, std::vector<std::string>& node_root, std::string node_name);
  void walkTree(std::string file, std::vector<std::string>& root, std::string name);
  void walkTree(std::string file, std::vector<std::string>& root);
  void walkTree(std::string file);

  void updateActiveName(std::string tag);

  void insertRegister(std::string key, std::string value);

  struct ParamsInitStage {
    std::string name;
    std::string duration;
    YAML::Node stage;
    YAML::Node costs;
    YAML::Node contacts;
  };

  std::string file_;
  std::string path_root_;
  std::vector<ParamsInitStage> stages_;
  std::stack<std::string> parsing_file_;
  std::map<std::string, std::string> params_;

  std::string active_name_;

  YAML::Node robot_;
};
}  // namespace multicopter_mpc

#endif