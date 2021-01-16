#include "yaml_parser/parser_yaml.h"

#include <string>
#include <vector>
#include <list>
#include <stack>
#include <regex>
#include <map>
#include <iostream>
#include <algorithm>
#include <numeric>

namespace {
//====== START OF FORWARD DECLARATION ========
std::string parseAtomicNode(YAML::Node);
std::string fetchMapEntry(YAML::Node);
std::string mapToString(std::map<std::string, std::string>);
//====== END OF FORWARD DECLARATION ========

/** @Brief Interprets a map as being atomic and thus parses it as a single entity. We assume that the map has as values
 * only scalars and sequences.
 *  @param n the node representing a map
 *  @return std::map<std::string, std::string> populated with the key,value pairs in n
 */
std::map<std::string, std::string> fetchAsMap(YAML::Node _n) {
  assert(_n.Type() == YAML::NodeType::Map && "trying to fetch as Map a non-Map node");
  auto m = std::map<std::string, std::string>();
  for (const auto& kv : _n) {
    std::string key = kv.first.as<std::string>();
    switch (kv.second.Type()) {
      case YAML::NodeType::Scalar: {
        std::string value = kv.second.Scalar();
        m.insert(std::pair<std::string, std::string>(key, value));
        break;
      }
      case YAML::NodeType::Sequence: {
        std::string aux = parseAtomicNode(kv.second);
        m.insert(std::pair<std::string, std::string>(key, aux));
        break;
      }
      case YAML::NodeType::Map: {
        std::string value = fetchMapEntry(kv.second);
        std::regex r("^\\$.*");
        if (std::regex_match(key, r)) key = key.substr(1, key.size() - 1);
        m.insert(std::pair<std::string, std::string>(key, value));
        break;
      }
      default:
        assert(1 == 0 && "Unsupported node Type at fetchAsMap");
        break;
    }
  }
  return m;
}
std::string fetchMapEntry(YAML::Node _n) {
  switch (_n.Type()) {
    case YAML::NodeType::Scalar: {
      return _n.Scalar();
      break;
    }
    case YAML::NodeType::Sequence: {
      return parseAtomicNode(_n);
      break;
    }
    case YAML::NodeType::Map: {
      return mapToString(fetchAsMap(_n));
      break;
    }
    default: {
      assert(1 == 0 && "Unsupported node Type at fetchMapEntry");
      return "";
      break;
    }
  }
}
/** @Brief Transforms a std::map<std::string,std::string> to its std::string representation
 * [{k1:v1},{k2:v2},{k3:v3},...]
 * @param map_ just a std::map<std::string,std::string>
 * @return <b>{std::string}</b> [{k1:v1},{k2:v2},{k3:v3},...]
 */
std::string mapToString(std::map<std::string, std::string> _map) {
  std::string result = "";
  auto v = std::vector<std::string>();
  std::transform(_map.begin(), _map.end(), back_inserter(v),
                 [](const std::pair<std::string, std::string> p) { return "{" + p.first + ":" + p.second + "}"; });
  auto concat = [](std::string ac, std::string str) -> std::string { return ac + str + ","; };
  std::string aux = "";
  std::string accumulated = std::accumulate(v.begin(), v.end(), aux, concat);
  if (accumulated.size() > 1)
    accumulated = accumulated.substr(0, accumulated.size() - 1);
  else
    accumulated = "";
  return "[" + accumulated + "]";
}
/** @Brief Generates a std::string representing a YAML sequence. The sequence is assumed to be scalar or at most be a
 * sequence of sequences of scalars.
 * @param n a vector of YAML::Node that represents a YAML::Sequence
 * @return <b>{std::string}</b> representing the YAML sequence
 */
std::string parseAtomicNode(YAML::Node _n) {
  std::string aux = "";
  bool first = true;
  std::string separator = "";
  switch (_n.Type()) {
    case YAML::NodeType::Scalar:
      return _n.Scalar();
      break;
    case YAML::NodeType::Sequence:
      for (auto it : _n) {
        aux += separator + parseAtomicNode(it);
        if (first) {
          separator = ",";
          first = false;
        }
      }
      return "[" + aux + "]";
      break;
    case YAML::NodeType::Map:
      return mapToString(fetchAsMap(_n));
      break;
    default:
      return "";
      break;
  }
}

/** @Brief checks if a node of the YAML tree is atomic. Only works if the nodes are of type
 * Scalar, Sequence or Map.
 * @param key is the key associated to the node n if n.Type() == YAML::NodeType::Map
 * @param n node to be test for atomicity
 */
bool isAtomic(std::string _key, YAML::Node _n) {
  assert(_n.Type() != YAML::NodeType::Undefined && _n.Type() != YAML::NodeType::Null &&
         "Cannot determine atomicity of Undefined/Null node");
  std::regex r("^\\$.*");
  bool is_atomic = true;
  switch (_n.Type()) {
    case YAML::NodeType::Scalar:
      return true;
      break;
    case YAML::NodeType::Sequence:
      for (auto it : _n) {
        switch (it.Type()) {
          case YAML::NodeType::Map:
            for (const auto& kv : it) {
              is_atomic = is_atomic and isAtomic(kv.first.as<std::string>(), it);
            }
            break;
          default:
            is_atomic = is_atomic and isAtomic("", it);
            break;
        }
      }
      return is_atomic;
      break;
    case YAML::NodeType::Map:
      is_atomic = std::regex_match(_key, r);
      return is_atomic;
      break;
    default:
      throw std::runtime_error("Cannot determine atomicity of node type " + std::to_string(_n.Type()));
      return false;
      break;
  }
  return false;
}
}  // namespace

namespace yaml_parser {
ParserYAML::ParserYAML(std::string _file, std::string path_root, bool freely_parse) {
  params_ = std::map<std::string, std::string>();
  active_name_ = "";
  paramsSens_ = std::vector<ParamsInitSensor>();
  paramsProc_ = std::vector<ParamsInitProcessor>();
  subscriber_managers_ = std::vector<SubscriberManager>();
  publisher_managers_ = std::vector<PublisherManager>();
  parsing_file_ = std::stack<std::string>();
  file_ = _file;
  if (path_root != "") {
    std::regex r(".*/ *$");
    if (not std::regex_match(path_root, r))
      path_root_ = path_root + "/";
    else
      path_root_ = path_root;
  }
  if (not freely_parse)
    parse();
  else
    parse_freely();
}

std::string ParserYAML::generatePath(std::string _file) {
  std::regex r("^/.*");
  if (std::regex_match(_file, r)) {
    return _file;
  } else {
    return path_root_ + _file;
  }
}
YAML::Node ParserYAML::loadYAML(std::string _file) {
  try {
    std::cout << "Parsing " << generatePath(_file) << std::endl;
    return YAML::LoadFile(generatePath(_file));
  } catch (YAML::BadFile& e) {
    throw std::runtime_error("Couldn't load file " + generatePath(_file) + ". Tried to open it from " +
                             parsing_file_.top());
  }
}
std::string ParserYAML::tagsToString(std::vector<std::string>& _tags) {
  std::string hdr = "";
  for (auto it : _tags) {
    hdr = hdr + "/" + it;
  }
  return hdr;
}
void ParserYAML::walkTree(std::string _file) {
  YAML::Node n;
  n = loadYAML(generatePath(_file));
  parsing_file_.push(generatePath(_file));
  std::vector<std::string> hdrs = std::vector<std::string>();
  walkTreeR(n, hdrs, "");
  parsing_file_.pop();
}
void ParserYAML::walkTree(std::string _file, std::vector<std::string>& _tags) {
  YAML::Node n;
  n = loadYAML(generatePath(_file));
  parsing_file_.push(generatePath(_file));
  walkTreeR(n, _tags, "");
  parsing_file_.pop();
}
void ParserYAML::walkTree(std::string _file, std::vector<std::string>& _tags, std::string hdr) {
  YAML::Node n;
  n = loadYAML(generatePath(_file));
  parsing_file_.push(generatePath(_file));
  walkTreeR(n, _tags, hdr);
  parsing_file_.pop();
}
void ParserYAML::walkTreeR(YAML::Node _n, std::vector<std::string>& _tags, std::string hdr) {
  switch (_n.Type()) {
    case YAML::NodeType::Scalar: {
      std::regex r("^@.*");
      if (std::regex_match(_n.Scalar(), r)) {
        std::string str = _n.Scalar();
        walkTree(str.substr(1, str.size() - 1), _tags, hdr);
      } else {
        insert_register(hdr, _n.Scalar());
      }
      break;
    }
    case YAML::NodeType::Sequence: {
      if (isAtomic("", _n)) {
        insert_register(hdr, parseAtomicNode(_n));
      } else {
        for (const auto& kv : _n) {
          walkTreeR(kv, _tags, hdr);
        }
      }
      break;
    }
    case YAML::NodeType::Map: {
      for (const auto& kv : _n) {
        if (isAtomic(kv.first.as<std::string>(), _n)) {
          std::string key = kv.first.as<std::string>();
          // WOLF_DEBUG("KEY IN MAP ATOMIC ", hdr + "/" + key);
          key = key.substr(1, key.size() - 1);
          insert_register(hdr + "/" + key, parseAtomicNode(kv.second));
        } else {
          /*
            If key=="follow" then the parser will assume that the value is a path and will parse
            the (expected) yaml file at the specified path. Note that this does not increase the header depth.
            The following example shows how the header remains unafected:
            @my_main_config                |  @some_path
            - cov_det: 1                   |  - my_value : 23
            - follow: "@some_path"         |
            - var: 1.2                     |
            Resulting map:
            cov_det -> 1
            my_value-> 23
            var: 1.2
            Instead of:
            cov_det -> 1
            follow/my_value-> 23
            var: 1.2
            Which would result from the following yaml files
            @my_main_config                |  @some_path
            - cov_det: 1                   |  - my_value : 23
            - $follow: "@some_path"        |
            - var: 1.2                     |
          */
          std::string key = kv.first.as<std::string>();
          // WOLF_DEBUG("KEY IN MAP NON ATOMIC ", key);
          std::regex rr("follow");
          if (not std::regex_match(kv.first.as<std::string>(), rr)) {
            _tags.push_back(kv.first.as<std::string>());
            if (_tags.size() == 2) updateActiveName(kv.first.as<std::string>());
            walkTreeR(kv.second, _tags, hdr + "/" + kv.first.as<std::string>());
            _tags.pop_back();
            if (_tags.size() == 1) updateActiveName("");
          } else {
            walkTree(kv.second.as<std::string>(), _tags, hdr);
          }
        }
      }
      break;
    }
    case YAML::NodeType::Null:
      break;
    default:
      assert(1 == 0 && "Unsupported node Type at walkTreeR.");
      break;
  }
}
void ParserYAML::updateActiveName(std::string _tag) { active_name_ = _tag; }
void ParserYAML::parseFirstLevel(std::string _file) {
  YAML::Node n;
  n = loadYAML(generatePath(_file));

  YAML::Node n_config = n["config"];
  // assert(n_config.Type() == YAML::NodeType::Map && "trying to parse config node but found a non-Map node");
  if (n_config.Type() != YAML::NodeType::Map)
    throw std::runtime_error("Could not find config node. Please make sure that your YAML file " +
                             generatePath(_file) + " starts with 'config:'");
  if (n_config["problem"].Type() != YAML::NodeType::Map)
    throw std::runtime_error("Could not find problem node. Please make sure that the 'config' node in YAML file " +
                             generatePath(_file) + " has a 'problem' entry");
  problem = n_config["problem"];
  std::vector<std::map<std::string, std::string>> map_container;
  try {
    for (const auto& kv : n_config["sensors"]) {
      ParamsInitSensor pSensor = {kv["type"].Scalar(), kv["name"].Scalar(), kv["plugin"].Scalar(), kv};
      paramsSens_.push_back(pSensor);
      map_container.push_back(std::map<std::string, std::string>(
          {{"type", kv["type"].Scalar()}, {"name", kv["name"].Scalar()}, {"plugin", kv["plugin"].Scalar()}}));
    }
    insert_register("sensors", yaml_parser::converter<std::string>::convert(map_container));
    map_container.clear();
  } catch (YAML::InvalidNode& e) {
    throw std::runtime_error("Error parsing sensors @" + generatePath(_file) +
                             ". Please make sure that each sensor entry has 'type', 'name' and 'plugin' entries.");
  }

  try {
    for (const auto& kv : n_config["processors"]) {
      ParamsInitProcessor pProc = {kv["type"].Scalar(), kv["name"].Scalar(), kv["sensor_name"].Scalar(),
                                   kv["plugin"].Scalar(), kv};
      paramsProc_.push_back(pProc);
      map_container.push_back(std::map<std::string, std::string>({{"type", kv["type"].Scalar()},
                                                                  {"name", kv["name"].Scalar()},
                                                                  {"plugin", kv["plugin"].Scalar()},
                                                                  {"sensor_name", kv["sensor_name"].Scalar()}}));
    }
    insert_register("processors", yaml_parser::converter<std::string>::convert(map_container));
    map_container.clear();
  } catch (YAML::InvalidNode& e) {
    throw std::runtime_error("Error parsing processors @" + generatePath(_file) +
                             ". Please make sure that each processor has 'type', 'name', 'plugin' and "
                             "'sensor_name' entries.");
  }
  try {
    for (const auto& kv : n_config["ROS subscriber"]) {
      SubscriberManager pSubscriberManager = {kv["package"].Scalar(), kv["type"].Scalar(), kv["topic"].Scalar(),
                                              kv["sensor_name"].Scalar(), kv};
      subscriber_managers_.push_back(pSubscriberManager);
      map_container.push_back(std::map<std::string, std::string>({{"package", kv["package"].Scalar()},
                                                                  {"type", kv["type"].Scalar()},
                                                                  {"topic", kv["topic"].Scalar()},
                                                                  {"sensor_name", kv["sensor_name"].Scalar()}}));
    }
    insert_register("ROS subscriber", yaml_parser::converter<std::string>::convert(map_container));
    map_container.clear();
  } catch (YAML::InvalidNode& e) {
    throw std::runtime_error("Error parsing subscriber @" + generatePath(_file) +
                             ". Please make sure that each manager has 'package', 'type', 'topic' and "
                             "'sensor_name' entries.");
  }

  try {
    for (const auto& kv : n_config["ROS publisher"]) {
      PublisherManager pPublisherManager = {kv["package"].Scalar(), kv["type"].Scalar(), kv["topic"].Scalar(),
                                            kv["period"].Scalar(), kv};
      publisher_managers_.push_back(pPublisherManager);
      map_container.push_back(std::map<std::string, std::string>({{"package", kv["package"].Scalar()},
                                                                  {"type", kv["type"].Scalar()},
                                                                  {"topic", kv["topic"].Scalar()},
                                                                  {"period", kv["period"].Scalar()}}));
    }
    insert_register("ROS publisher", yaml_parser::converter<std::string>::convert(map_container));
    map_container.clear();
  } catch (YAML::InvalidNode& e) {
    throw std::runtime_error(
        "Error parsing publisher @" + generatePath(_file) +
        ". Please make sure that each manager has 'package', 'type', 'topic' and 'period' entries.");
  }
}
std::map<std::string, std::string> ParserYAML::getParams() {
  std::map<std::string, std::string> rtn = params_;
  return rtn;
}
void ParserYAML::parse() {
  parsing_file_.push(generatePath(file_));
  parseFirstLevel(file_);

  if (problem.Type() != YAML::NodeType::Undefined) {
    std::vector<std::string> tags = std::vector<std::string>();
    walkTreeR(problem, tags, "problem");
  }
  for (auto it : paramsSens_) {
    std::vector<std::string> tags = std::vector<std::string>();
    tags.push_back("sensor");
    walkTreeR(it.n_, tags, "sensor/" + it.name_);
  }
  for (auto it : paramsProc_) {
    std::vector<std::string> tags = std::vector<std::string>();
    tags.push_back("processor");
    walkTreeR(it.n_, tags, "processor/" + it.name_);
  }
  std::list<std::string> plugins;
  std::list<std::string> packages_subscriber, packages_publisher;
  for (const auto& it : paramsSens_) plugins.push_back(it.plugin_);
  for (const auto& it : paramsProc_) plugins.push_back(it.plugin_);
  for (const auto& it : subscriber_managers_) packages_subscriber.push_back(it.package_);
  for (const auto& it : publisher_managers_) packages_publisher.push_back(it.package_);
  plugins.sort();
  plugins.unique();
  packages_subscriber.sort();
  packages_subscriber.unique();
  packages_publisher.sort();
  packages_publisher.unique();
  insert_register("plugins", yaml_parser::converter<std::string>::convert(plugins));
  insert_register("packages_subscriber", yaml_parser::converter<std::string>::convert(packages_subscriber));
  insert_register("packages_publisher", yaml_parser::converter<std::string>::convert(packages_publisher));

  YAML::Node n;
  n = loadYAML(generatePath(file_));

  if (n.Type() == YAML::NodeType::Map) {
    for (auto it : n) {
      auto node = it.second;
      // WOLF_INFO("WUT ", it.first);
      std::vector<std::string> tags = std::vector<std::string>();
      if (it.first.as<std::string>() != "config")
        walkTreeR(node, tags, it.first.as<std::string>());
      else {
        for (auto itt : node) {
          std::string node_key = itt.first.as<std::string>();
          if (node_key != "problem" and node_key != "sensors" and node_key != "processors" and
              node_key != "ROS subscriber" and node_key != "ROS publisher") {
            walkTreeR(itt.second, tags, node_key);
          }
        }
      }
    }
  } else {
    std::vector<std::string> tags = std::vector<std::string>();
    walkTreeR(n, tags, "");
  }
  parsing_file_.pop();
}
void ParserYAML::parse_freely() {
  parsing_file_.push(generatePath(file_));
  std::vector<std::string> tags = std::vector<std::string>();
  walkTreeR(loadYAML(file_), tags, "");
  parsing_file_.pop();
}
void ParserYAML::insert_register(std::string _key, std::string _value) {
  if (_key.substr(0, 1) == "/") _key = _key.substr(1, _key.size() - 1);
  auto inserted_it = params_.insert(std::pair<std::string, std::string>(_key, _value));
  if (not inserted_it.second)
    std::cout << "Skipping key '" << _key << "' with value '" << _value << "'. There already exists the register: ("
              << inserted_it.first->first << "," << inserted_it.first->second << ")" << std::endl;
}
}  // namespace yaml_parser