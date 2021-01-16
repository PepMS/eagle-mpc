#ifndef MULTICOPTER_MPC_UTILS_PARSER_YAML_HPP_
#define MULTICOPTER_MPC_UTILS_PARSER_YAML_HPP_

#include <string>
#include <regex>

#include "yaml-cpp/yaml.h"

namespace multicopter_mpc {
class ParserYaml {
 public:
  ParserYaml(std::string file_path);

 private:
  void parse();
  void parseFirstLevel(const std::string& file_path);
  YAML::Node loadYaml(const std::string file_path);
  void walkTreeRecursive(const YAML::Node& node, const std::string& node_root, const std::string& node_name);
  
  struct ParamsInitStage {
    std::string name;
    std::string duration;
    YAML::Node n;
  };

  std::string file_path_;
  std::vector<ParamsInitStage> stages_;

  YAML::Node robot_;
};
}  // namespace multicopter_mpc

#endif