#ifndef PARSER_YAML_HPP
#define PARSER_YAML_HPP

#include "yaml_parser/converter.h"

#include "yaml-cpp/yaml.h"

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
  std::string mapToString(std::map<std::string,std::string>);
  //====== END OF FORWARD DECLARATION ========

/** @Brief Interprets a map as being atomic and thus parses it as a single entity. We assume that the map has as values only scalars and sequences.
 *  @param n the node representing a map
 *  @return std::map<std::string, std::string> populated with the key,value pairs in n
 */
std::map<std::string, std::string> fetchAsMap(YAML::Node n){
    assert(n.Type() == YAML::NodeType::Map && "trying to fetch as Map a non-Map node");
    auto m = std::map<std::string, std::string>();
    for(const auto& kv : n){
        std::string key = kv.first.as<std::string>();
        switch (kv.second.Type()) {
        case YAML::NodeType::Scalar : {
            std::string value = kv.second.Scalar();
            m.insert(std::pair<std::string,std::string>(key, value));
            break;
        }
        case YAML::NodeType::Sequence : {
            std::string aux = parseAtomicNode(kv.second);
            m.insert(std::pair<std::string,std::string>(key, aux));
            break;
        }
        case YAML::NodeType::Map : {
          std::string value = fetchMapEntry(kv.second);
          std::regex r("^\\$.*");
          if (std::regex_match(key, r)) key = key.substr(1,key.size()-1);
          m.insert(std::pair<std::string,std::string>(key, value));
          break;
        }
        default:
            assert(1 == 0 && "Unsupported node Type at fetchAsMap");
            break;
        }
    }
    return m;
}
  std::string fetchMapEntry(YAML::Node n){
    switch (n.Type()) {
    case YAML::NodeType::Scalar: {
      return n.Scalar();
      break;
    }
    case YAML::NodeType::Sequence: {
      return parseAtomicNode(n);
      break;
    }
    case YAML::NodeType::Map: {
      return mapToString(fetchAsMap(n));
      break;
    }
    default: {
      assert(1 == 0 && "Unsupported node Type at fetchMapEntry");
      return "";
      break;
    }
    }
  }
    /** @Brief Transforms a std::map<std::string,std::string> to its std::string representation [{k1:v1},{k2:v2},{k3:v3},...]
    * @param _map just a std::map<std::string,std::string>
    * @return <b>{std::string}</b> [{k1:v1},{k2:v2},{k3:v3},...]
    */
    std::string mapToString(std::map<std::string,std::string> _map){
        std::string result = "";
        auto v = std::vector<std::string>();
        std::transform(_map.begin(), _map.end(), back_inserter(v), [](const std::pair<std::string,std::string> p){return "{" + p.first + ":" + p.second + "}";});
        auto concat = [](std::string ac, std::string str)-> std::string {
                          return ac + str + ",";
                      };
        std::string aux = "";
        std::string accumulated = std::accumulate(v.begin(), v.end(), aux, concat);
        if(accumulated.size() > 1) accumulated = accumulated.substr(0,accumulated.size() - 1);
        else accumulated = "";
        return "[" + accumulated + "]";
    }
    /** @Brief Generates a std::string representing a YAML sequence. The sequence is assumed to be scalar or at most be a sequence of sequences of scalars.
    * @param n a vector of YAML::Node that represents a YAML::Sequence
    * @return <b>{std::string}</b> representing the YAML sequence
    */
    std::string parseAtomicNode(YAML::Node n){
      std::string aux = "";
      bool first = true;
      std::string separator = "";
      switch(n.Type()){
      case YAML::NodeType::Scalar:
        return n.Scalar();
        break;
      case YAML::NodeType::Sequence:
        for(auto it : n){
          aux += separator + parseAtomicNode(it);
          if(first){
            separator = ",";
            first = false;
          }
        }
        return "[" + aux + "]";
        break;
      case YAML::NodeType::Map:
        return mapToString(fetchAsMap(n));
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
    bool isAtomic(std::string key, YAML::Node n){
      assert(n.Type() != YAML::NodeType::Undefined && n.Type() != YAML::NodeType::Null && "Cannot determine atomicity of Undefined/Null node");
      std::regex r("^\\$.*");
      bool is_atomic = true;
      switch(n.Type()){
      case YAML::NodeType::Scalar:
        return true;
        break;
      case YAML::NodeType::Sequence:
        for(auto it : n) {
          switch(it.Type()){
          case YAML::NodeType::Map:
            for(const auto& kv : it){
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
        is_atomic = std::regex_match(key, r);
        return is_atomic;
        break;
      default:
        throw std::runtime_error("Cannot determine atomicity of node type " + std::to_string(n.Type()));
        return false;
        break;
      }
      return false;
    }
}
namespace yaml_parser {
class ParserYAML {
    struct ParamsInitSensor{
        std::string _type;
        std::string _name;
        std::string _plugin;
        YAML::Node n;
    };
    struct ParamsInitProcessor{
        std::string _type;
        std::string _name;
        std::string _name_assoc_sensor;
        std::string _plugin;
        YAML::Node n;
    };
    struct SubscriberManager{
        std::string _package;
        std::string _subscriber;
        std::string _topic;
        std::string _sensor_name;
        YAML::Node n;
    };
    std::map<std::string, std::string> _params;
    std::string _active_name;
    std::vector<ParamsInitSensor> _paramsSens;
    std::vector<ParamsInitProcessor> _paramsProc;
    std::stack<std::string> _parsing_file;
    std::string _file;
    std::string _path_root;
    std::vector<SubscriberManager> _subscriber_managers;
    YAML::Node problem;
    std::string generatePath(std::string);
    YAML::Node loadYAML(std::string);
    void insert_register(std::string, std::string);
public:
    ParserYAML(std::string file, std::string path_root = "",
               bool freely_parse = false) {
      _params = std::map<std::string, std::string>();
      _active_name = "";
      _paramsSens = std::vector<ParamsInitSensor>();
      _paramsProc = std::vector<ParamsInitProcessor>();
      _subscriber_managers = std::vector<SubscriberManager>();
      _parsing_file = std::stack<std::string>();
      _file = file;
      if (path_root != "") {
        std::regex r(".*/ *$");
        if (not std::regex_match(path_root, r))
          _path_root = path_root + "/";
        else
          _path_root = path_root;
      }
      if(not freely_parse) this->parse();
      else this->parse_freely();
    }
    ~ParserYAML()
    {
        //
    }
    void parse_freely();
    std::map<std::string, std::string> getParams();

  private:
    void walkTree(std::string file);
    void walkTree(std::string file, std::vector<std::string>& tags);
    void walkTree(std::string file, std::vector<std::string>& tags, std::string hdr);
    void walkTreeR(YAML::Node n, std::vector<std::string>& tags, std::string hdr);
    void updateActiveName(std::string tag);
    void parseFirstLevel(std::string file);
    std::string tagsToString(std::vector<std::string>& tags);
    void parse();
};
std::string ParserYAML::generatePath(std::string file){
    std::regex r("^/.*");
    if(std::regex_match(file, r)){
        return file;
    }else{
        return _path_root + file;
    }
}
YAML::Node ParserYAML::loadYAML(std::string file){
  try{
    // std::cout << "Parsing " << generatePath(file) << std::endl;
    // WOLF_INFO("Parsing file: ", generatePath(file));
    return YAML::LoadFile(generatePath(file));
  }catch (YAML::BadFile& e){
    throw std::runtime_error("Couldn't load file " + generatePath(file) + ". Tried to open it from " + this->_parsing_file.top());
  }
}
std::string ParserYAML::tagsToString(std::vector<std::string> &tags){
    std::string hdr = "";
    for(auto it : tags){
        hdr = hdr + "/" + it;
    }
    return hdr;
}
void ParserYAML::walkTree(std::string file){
    YAML::Node n;
    n = loadYAML(generatePath(file));
    this->_parsing_file.push(generatePath(file));
    std::vector<std::string> hdrs = std::vector<std::string>();
    walkTreeR(n, hdrs, "");
    this->_parsing_file.pop();
}
void ParserYAML::walkTree(std::string file, std::vector<std::string>& tags){
    YAML::Node n;
    n = loadYAML(generatePath(file));
    this->_parsing_file.push(generatePath(file));
    walkTreeR(n, tags, "");
    this->_parsing_file.pop();
}
void ParserYAML::walkTree(std::string file, std::vector<std::string>& tags, std::string hdr){
    YAML::Node n;
    n = loadYAML(generatePath(file));
    this->_parsing_file.push(generatePath(file));
    walkTreeR(n, tags, hdr);
    this->_parsing_file.pop();
}
/** @Brief Recursively walks the YAML tree while filling a map with the values parsed from the file
* @param YAML node to be parsed
* @param tags represents the path from the root of the YAML tree to the current node
* @param hdr is the name of the current YAML node
*/
void ParserYAML::walkTreeR(YAML::Node n, std::vector<std::string>& tags, std::string hdr){
    switch (n.Type()) {
    case YAML::NodeType::Scalar : {
        std::regex r("^@.*");
        if(std::regex_match(n.Scalar(), r)){
            std::string str = n.Scalar();
            walkTree(str.substr(1,str.size() - 1), tags, hdr);
        }else{
          insert_register(hdr, n.Scalar());
        }
        break;
    }
    case YAML::NodeType::Sequence : {
      if(isAtomic("", n)){
        insert_register(hdr, parseAtomicNode(n));
        }else{
          for(const auto& kv : n){
            walkTreeR(kv, tags, hdr);
          }
        }
        break;
    }
    case YAML::NodeType::Map : {
      for(const auto& kv : n){
        if(isAtomic(kv.first.as<std::string>(), n)){
          std::string key = kv.first.as<std::string>();
          //WOLF_DEBUG("KEY IN MAP ATOMIC ", hdr + "/" + key);
          key = key.substr(1,key.size() - 1);
          insert_register(hdr + "/" + key, parseAtomicNode(kv.second));
      }else{
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
          //WOLF_DEBUG("KEY IN MAP NON ATOMIC ", key);
          std::regex rr("follow");
          if(not std::regex_match(kv.first.as<std::string>(), rr)) {
            tags.push_back(kv.first.as<std::string>());
            if(tags.size() == 2) this->updateActiveName(kv.first.as<std::string>());
            walkTreeR(kv.second, tags, hdr +"/"+ kv.first.as<std::string>());
            tags.pop_back();
            if(tags.size() == 1) this->updateActiveName("");
          }else{
            walkTree(kv.second.as<std::string>(), tags, hdr);
          }
      }
      }
        break;
    }
    default:
        assert(1 == 0 && "Unsupported node Type at walkTreeR.");
        break;
    }
}
void ParserYAML::updateActiveName(std::string tag){
    this->_active_name = tag;
}
/** @Brief Parse the sensors, processors, callbacks and files elements of the YAML file. We assume that these elements
    are defined at the top level of the YAML file.
 * @param file is the path to the YAML file */
void ParserYAML::parseFirstLevel(std::string file){
    YAML::Node n;
    n = loadYAML(generatePath(file));

    YAML::Node n_config = n["config"];
    // assert(n_config.Type() == YAML::NodeType::Map && "trying to parse config node but found a non-Map node");
    if(n_config.Type() != YAML::NodeType::Map) throw std::runtime_error("Could not find config node. Please make sure that your YAML file " + generatePath(file) + " starts with 'config:'");
    if(n_config["problem"].Type() != YAML::NodeType::Map) throw std::runtime_error("Could not find problem node. Please make sure that the 'config' node in YAML file " + generatePath(file) + " has a 'problem' entry");
    this->problem = n_config["problem"];
    std::vector<std::map<std::string, std::string>> map_container;
    try{
      for(const auto& kv : n_config["sensors"]){
          ParamsInitSensor pSensor = {kv["type"].Scalar(), kv["name"].Scalar(), kv["plugin"].Scalar(), kv};
          _paramsSens.push_back(pSensor);
          map_container.push_back(std::map<std::string, std::string>({
                                                                      {"type", kv["type"].Scalar()},
                                                                      {"name", kv["name"].Scalar()},
                                                                      {"plugin", kv["plugin"].Scalar()}
                  }));
      }
      insert_register("sensors", yaml_parser::converter<std::string>::convert(map_container));
      map_container.clear();
    } catch(YAML::InvalidNode& e){
      throw std::runtime_error("Error parsing sensors @" + generatePath(file) + ". Please make sure that each sensor entry has 'type', 'name' and 'plugin' entries.");
    }

    try{
      for(const auto& kv : n_config["processors"]){
          ParamsInitProcessor pProc = {kv["type"].Scalar(), kv["name"].Scalar(), kv["sensor_name"].Scalar(), kv["plugin"].Scalar(), kv};
          _paramsProc.push_back(pProc);
          map_container.push_back(std::map<std::string, std::string>({
                                                                      {"type", kv["type"].Scalar()},
                                                                      {"name", kv["name"].Scalar()},
                                                                      {"plugin", kv["plugin"].Scalar()},
                                                                      {"sensor_name", kv["sensor_name"].Scalar()}}));
      }
      insert_register("processors",
                      yaml_parser::converter<std::string>::convert(map_container));
      map_container.clear();
    } catch(YAML::InvalidNode& e){
      throw std::runtime_error("Error parsing processors @" + generatePath(file) + ". Please make sure that each processor has 'type', 'name', 'plugin' and 'sensor_name' entries.");
    }
    try {
      for (const auto &kv : n_config["ROS subscriber managers"]) {
          SubscriberManager pSubscriberManager = {kv["package"].Scalar(), kv["type"].Scalar(), kv["topic"].Scalar(), kv["sensor_name"].Scalar(), kv};
          _subscriber_managers.push_back(pSubscriberManager);
          map_container.push_back(std::map<std::string, std::string>({
                                                                      {"package", kv["package"].Scalar()},
                                                                      {"type", kv["type"].Scalar()},
                                                                      {"topic", kv["topic"].Scalar()},
                                                                      {"sensor_name", kv["sensor_name"].Scalar()}}));
      }
      insert_register("ROS subscriber managers", yaml_parser::converter<std::string>::convert(map_container));
      map_container.clear();
    } catch (YAML::InvalidNode &e) {
        throw std::runtime_error("Error parsing subscriber managers @" + generatePath(file) + ". Please make sure that each manager has 'package', 'type', 'topic' and 'sensor_name' entries.");
    }
}
std::map<std::string,std::string> ParserYAML::getParams(){
    std::map<std::string,std::string> rtn = _params;
    return rtn;
}
void ParserYAML::parse(){
    this->_parsing_file.push(generatePath(this->_file));
    this->parseFirstLevel(this->_file);

    if(this->problem.Type() != YAML::NodeType::Undefined){
      std::vector<std::string> tags = std::vector<std::string>();
      this->walkTreeR(this->problem, tags , "problem");
    }
    for(auto it : _paramsSens){
      std::vector<std::string> tags = std::vector<std::string>();
      tags.push_back("sensor");
      this->walkTreeR(it.n , tags , "sensor/" + it._name);
    }
    for(auto it : _paramsProc){
      std::vector<std::string> tags = std::vector<std::string>();
      tags.push_back("processor");
      this->walkTreeR(it.n , tags , "processor/" + it._name);
    }
    std::list<std::string> plugins, packages;
    for (const auto& it : this->_paramsSens)
        plugins.push_back(it._plugin);
    for (const auto& it : this->_paramsProc)
        plugins.push_back(it._plugin);
    for (const auto& it : this->_subscriber_managers)
        packages.push_back(it._package);
    plugins.sort();
    plugins.unique();
    packages.sort();
    packages.unique();
    insert_register("plugins", yaml_parser::converter<std::string>::convert(plugins));
    insert_register("packages", yaml_parser::converter<std::string>::convert(packages));

    YAML::Node n;
    n = loadYAML(generatePath(this->_file));

    if (n.Type() == YAML::NodeType::Map)
        {
            for (auto it : n)
                {
                    auto node = it.second;
                    // WOLF_INFO("WUT ", it.first);
                    std::vector<std::string> tags = std::vector<std::string>();
                    if(it.first.as<std::string>() != "config")
                        this->walkTreeR(node, tags, it.first.as<std::string>());
                    else
                        {
                            for (auto itt : node)
                                {
                                    std::string node_key = itt.first.as<std::string>();
                                    // WOLF_INFO("node_key ", node_key);
                                    if (node_key != "problem" and node_key != "sensors" and node_key != "processors" and
                                        node_key != "ROS subscriber managers")
                                        {
                                            this->walkTreeR(itt.second, tags, node_key);
                                        }
                                }
                        }
                }
        }else
        {
            std::vector<std::string> tags = std::vector<std::string>();
            this->walkTreeR(n, tags, "");
        }
    this->_parsing_file.pop();
    }
void ParserYAML::parse_freely(){
    this->_parsing_file.push(generatePath(this->_file));
    std::vector<std::string> tags = std::vector<std::string>();
    this->walkTreeR(loadYAML(this->_file), tags, "");
    this->_parsing_file.pop();
}
void ParserYAML::insert_register(std::string key, std::string value){
    if(key.substr(0,1) == "/") key = key.substr(1,key.size()-1);
  auto inserted_it = _params.insert(std::pair<std::string, std::string>(key, value));
  // if(not inserted_it.second) WOLF_WARN("Skipping key '", key, "' with value '", value, "'. There already exists the register: (", inserted_it.first->first, ",", inserted_it.first->second, ")");
}
} // namespace yaml_parser
#endif
