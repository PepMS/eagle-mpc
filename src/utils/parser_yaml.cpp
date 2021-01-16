#include "multicopter_mpc/utils/parser_yaml.hpp"
#include "multicopter_mpc/utils/log.hpp"

namespace multicopter_mpc {
ParserYaml::ParserYaml(std::string file_path) {
  file_path_ = file_path;
  parse();
}

void ParserYaml::parse() {
  parseFirstLevel(file_path_);

  for (auto stage : stages_) {
    walkTreeRecursive(stage.n, "stages", "stages/" + stage.name);
  }
}

void ParserYaml::parseFirstLevel(const std::string& file_path) {
  YAML::Node n;
  n = loadYaml(file_path);

  YAML::Node n_trajectory = n["trajectory"];
  if (n_trajectory.Type() != YAML::NodeType::Map) {
    throw std::runtime_error("Could not find trajectory node. Please make sure that your YAML file " + file_path +
                             " starts with 'trajectory:'");
  }

  if (n_trajectory["robot"].Type() != YAML::NodeType::Map) {
    throw std::runtime_error("Could not find robot node. Please make sure that the 'trajectory' node in YAML file " +
                             file_path + " has a 'robot' entry");
  }
  robot_ = n_trajectory["robot"];

  try {
    for (auto stage : n_trajectory["stages"]) {
      ParamsInitStage p_stage = {stage["name"].Scalar(), stage["duration"].Scalar(), stage};
      stages_.push_back(p_stage);
    }
  } catch (const std::exception& e) {
    throw std::runtime_error("Error parsing stages @" + file_path);
  }
}

YAML::Node ParserYaml::loadYaml(const std::string file_path) {
  try {
    MMPC_INFO << "Parsing " << file_path;
    return YAML::LoadFile(file_path);
  } catch (YAML::BadFile& e) {
    throw std::runtime_error("Couldn't load file: " + file_path);
  }
}

void walkTreeRecursive(const YAML::Node& node, const std::string& node_root, const std::string& node_name) {
  switch (node.Type()) {
    case YAML::NodeType::Scalar: {
      std::regex r("^@.*");
      if (std::regex_match(node.Scalar(), r)) {
        std::string str = node.Scalar();
        // walkTree(str.substr(1, str.size() - 1), _tags, hdr);
      } else {
        std::cout << "Inside Scalar" << std::endl;
        // insert_register(hdr, _n.Scalar());
      }
      break;
    }
    default:
      MMPC_ERROR << "Unsupported node Type at walkTreeR.";
      break;
  }
}

}  // namespace multicopter_mpc