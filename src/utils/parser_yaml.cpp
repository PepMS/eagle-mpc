///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2021, Institut de Robotica i Informatica Industrial (CSIC-UPC)
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#include "eagle_mpc/utils/parser_yaml.hpp"
#include "eagle_mpc/utils/log.hpp"
#include "eagle_mpc/path.hpp"

namespace eagle_mpc
{
//====== START OF FORWARD DECLARATION ========
std::string parseAtomicNode(YAML::Node);
std::string fetchMapEntry(YAML::Node);
std::string mapToString(std::map<std::string, std::string>);
//====== END OF FORWARD DECLARATION ========

std::map<std::string, std::string> fetchAsMap(YAML::Node node)
{
    assert(node.Type() == YAML::NodeType::Map && "trying to fetch as Map a non-Map node");
    auto m = std::map<std::string, std::string>();

    for (const auto& kv : node) {
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
                std::regex  r("^\\$.*");
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

std::string fetchMapEntry(YAML::Node node)
{
    switch (node.Type()) {
        case YAML::NodeType::Scalar: {
            return node.Scalar();
            break;
        }
        case YAML::NodeType::Sequence: {
            return parseAtomicNode(node);
            break;
        }
        case YAML::NodeType::Map: {
            return mapToString(fetchAsMap(node));
            break;
        }
        default: {
            assert(1 == 0 && "Unsupported node Type at fetchMapEntry");
            return "";
            break;
        }
    }
}

std::string mapToString(std::map<std::string, std::string> map)
{
    std::string result = "";
    auto        v      = std::vector<std::string>();
    std::transform(map.begin(), map.end(), back_inserter(v),
                   [](const std::pair<std::string, std::string> p) { return "{" + p.first + ":" + p.second + "}"; });
    auto        concat      = [](std::string ac, std::string str) -> std::string { return ac + str + ","; };
    std::string aux         = "";
    std::string accumulated = std::accumulate(v.begin(), v.end(), aux, concat);
    if (accumulated.size() > 1)
        accumulated = accumulated.substr(0, accumulated.size() - 1);
    else
        accumulated = "";
    return "[" + accumulated + "]";
}

std::string parseAtomicNode(YAML::Node node)
{
    std::string aux       = "";
    bool        first     = true;
    std::string separator = "";
    switch (node.Type()) {
        case YAML::NodeType::Scalar:
            return node.Scalar();
            break;
        case YAML::NodeType::Sequence:
            for (auto it : node) {
                aux += separator + parseAtomicNode(it);
                if (first) {
                    separator = ",";
                    first     = false;
                }
            }
            return "[" + aux + "]";
            break;
        case YAML::NodeType::Map:
            return mapToString(fetchAsMap(node));
            break;
        default:
            return "";
            break;
    }
}

bool isAtomic(std::string key, YAML::Node node)
{
    assert(node.Type() != YAML::NodeType::Undefined && node.Type() != YAML::NodeType::Null &&
           "Cannot determine atomicity of Undefined/Null node");
    std::regex r("^\\$.*");
    bool       is_atomic = true;

    switch (node.Type()) {
        case YAML::NodeType::Scalar:
            return true;
            break;
        case YAML::NodeType::Sequence:
            for (auto it : node) {
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
            is_atomic = std::regex_match(key, r);
            return is_atomic;
            break;
        default:
            throw std::runtime_error("Cannot determine atomicity of node type " + std::to_string(node.Type()));
            return false;
            break;
    }
    return false;
}

std::string getYamlPath(const std::string& yaml_path)
{
    std::string path;
    path = yaml_path.find("/", 0) == 0 ? yaml_path : std::string(EAGLE_MPC_YAML_DIR) + "/" + yaml_path;
    return path;
}

std::string getUrdfPath(const std::string& urdf_path)
{
    std::string path;
    path = urdf_path.find("/", 0) == 0 ? urdf_path : std::string(EAGLE_MPC_ROBOT_DATA_DIR) + "/" + urdf_path;
    return path;
}

ParserYaml::ParserYaml(std::string file, std::string path_root, const bool& freely_parse)
{
    file_         = file;
    parsing_file_ = std::stack<std::string>();

    if (path_root != "") {
        std::regex r(".*/ *$");
        if (not std::regex_match(path_root, r))
            path_root_ = path_root + "/";
        else
            path_root_ = path_root;
    }
    if (not freely_parse) {
        parse();
    } else {
        parseFreely();
    }
}

void ParserYaml::parse()
{
    parsing_file_.push(generatePath(file_));
    parseFirstLevel(file_);

    if (robot_.Type() != YAML::NodeType::Undefined) {
        std::vector<std::string> tags = std::vector<std::string>();
        walkTreeRecursive(robot_, tags, "robot");
    }

    if (is_trajectory_) {
        if (problem_params_.Type() == YAML::NodeType::Map) {
            std::vector<std::string> tags = std::vector<std::string>();
            walkTreeRecursive(problem_params_, tags, "problem_params");
        }
        for (auto stage : stages_) {
            std::vector<std::string> tags = std::vector<std::string>();
            insertRegister("stages/" + stage.name + "/name", stage.name);
            insertRegister("stages/" + stage.name + "/duration", stage.duration);
            insertRegister("stages/" + stage.name + "/transition", stage.transition);
            for (auto cost : stage.costs) {
                tags.push_back("stages/" + stage.name + "/costs");
                walkTreeRecursive(cost, tags, "stages/" + stage.name + "/costs/" + cost["name"].Scalar());
            }
            for (auto contact : stage.contacts) {
                tags.push_back("stages/" + stage.name + "/contacts");
                walkTreeRecursive(contact, tags, "stages/" + stage.name + "/contacts/" + contact["name"].Scalar());
            }
        }
    }
}

void ParserYaml::parseFirstLevel(std::string file)
{
    YAML::Node n;
    n = loadYaml(generatePath(file));

    YAML::Node n_trajectory = n["trajectory"];
    if (n_trajectory.Type() != YAML::NodeType::Map) {
        YAML::Node n_mpc_controller = n["mpc_controller"];
        if (n_mpc_controller.Type() != YAML::NodeType::Map) {
            throw std::runtime_error(
                "Could not find neither a trajectory or an mpc_controller node. Please make sure that your YAML "
                "file " +
                generatePath(file) + " starts with 'trajectory:' or 'mpc_controller:'");
        } else {
            is_trajectory_ = false;
            parseMpcController(n_mpc_controller, file);
        }
    } else {
        is_trajectory_ = true;
        parseTrajectory(n_trajectory, file);
    }
}

void ParserYaml::parseTrajectory(const YAML::Node& node, const std::string& file)
{
    if (node["robot"].Type() != YAML::NodeType::Map) {
        throw std::runtime_error(
            "Could not find robot node. Please make sure that the 'trajectory' node in YAML file " +
            generatePath(file) + " has a 'robot' entry");
    }

    robot_ = node["robot"];

    try {
        if (node["problem_params"].Type() == YAML::NodeType::Map) {
            problem_params_ = node["problem_params"];
        }
    } catch (const std::exception& e) {
    }

    try {
        if (node["initial_state"].Type() == YAML::NodeType::Sequence) {
            insertRegister("initial_state", parseAtomicNode(node["initial_state"]));
        }
    } catch (const std::exception& e) {
    }

    std::vector<std::map<std::string, std::string>> map_container;
    try {
        for (auto stage : node["stages"]) {
            std::string transition;
            if (stage["transition"].Type() == YAML::NodeType::Undefined) {
                transition = "false";
            } else {
                transition = "true";
            }

            ParamsInitStage p_stage = {stage["name"].Scalar(), stage["duration"].Scalar(), transition, stage,
                                       stage["costs"],         stage["contacts"]};

            stages_.push_back(p_stage);

            std::vector<std::string> cost_container;
            for (auto cost : p_stage.costs) {
                cost_container.push_back(cost["name"].Scalar());
            }

            std::vector<std::string> contact_container;
            for (auto contact : p_stage.contacts) {
                contact_container.push_back(contact["name"].Scalar());
            }

            if (p_stage.contacts.Type() == YAML::NodeType::Undefined) {
                map_container.push_back(
                    std::map<std::string, std::string>({{"name", stage["name"].Scalar()},
                                                        {"duration", stage["duration"].Scalar()},
                                                        {"transition", transition},
                                                        {"costs", converter<std::string>::convert(cost_container)}}));
            } else {
                map_container.push_back(std::map<std::string, std::string>(
                    {{"name", stage["name"].Scalar()},
                     {"duration", stage["duration"].Scalar()},
                     {"transition", transition},
                     {"costs", converter<std::string>::convert(cost_container)},
                     {"contacts", converter<std::string>::convert(contact_container)}}));
            }
        }
        insertRegister("stages", converter<std::string>::convert(map_container));
    } catch (const std::exception& e) {
        throw std::runtime_error("Error parsing stages @" + generatePath(file) +
                                 ". Make sure every stage has a name, duration and, at least, one cost.");
    }
}

void ParserYaml::parseMpcController(const YAML::Node& node, const std::string& file)
{
    if (node["robot"].Type() != YAML::NodeType::Map) {
        throw std::runtime_error(
            "Could not find robot node. Please make sure that the 'trajectory' node in YAML file " +
            generatePath(file) + " has a 'robot' entry");
    }

    std::string mpc_prefix = "mpc_controller/";
    for (auto n = node.begin(); n != node.end(); ++n) {
        if (n->first.as<std::string>() == "robot") {
            robot_ = n->second.as<YAML::Node>();
        } else {
            insertRegister(mpc_prefix + n->first.as<std::string>(), parseAtomicNode(n->second));
        }
    }
}

void ParserYaml::parseFreely()
{
    parsing_file_.push(generatePath(file_));
    std::vector<std::string> tags = std::vector<std::string>();
    walkTreeRecursive(loadYaml(file_), tags, "");
    parsing_file_.pop();
}

YAML::Node ParserYaml::loadYaml(std::string file_path)
{
    try {
        EMPC_INFO("Parsing ", file_path);
        return YAML::LoadFile(file_path);
    } catch (YAML::BadFile& e) {
        throw std::runtime_error("Couldn't load file: " + file_path);
    }
}

std::string ParserYaml::generatePath(std::string file)
{
    std::regex r("^/.*");
    if (std::regex_match(file, r)) {
        return file;
    } else {
        return path_root_ + file;
    }
}

void ParserYaml::walkTreeRecursive(YAML::Node node, std::vector<std::string>& node_root, std::string node_name)
{
    switch (node.Type()) {
        case YAML::NodeType::Scalar: {
            std::regex r("^@.*");
            if (std::regex_match(node.Scalar(), r)) {
                std::string str = node.Scalar();
                walkTree(str.substr(1, str.size() - 1), node_root, node_name);
            } else {
                insertRegister(node_name, node.Scalar());
            }
            break;
        }
        case YAML::NodeType::Sequence: {
            if (isAtomic("", node)) {
                insertRegister(node_name, parseAtomicNode(node));
            } else {
                for (const auto& kv : node) {
                    walkTreeRecursive(kv, node_root, node_name);
                }
            }
            break;
        }
        case YAML::NodeType::Map: {
            for (const auto& kv : node) {
                if (isAtomic(kv.first.as<std::string>(), node)) {
                    std::string key = kv.first.as<std::string>();
                    key             = key.substr(1, key.size() - 1);
                    insertRegister(node_name + "/" + key, parseAtomicNode(kv.second));
                } else {
                    /*
                      If key=="follow" then the parser will assume that the value is a path and will parse
                      the (expected) yaml file at the specified path. Note that this does not increase the header
                      depth. The following example shows how the header remains unafected:
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
                    std::regex  rr("follow");

                    if (not std::regex_match(kv.first.as<std::string>(), rr)) {
                        node_root.push_back(kv.first.as<std::string>());
                        if (node_root.size() == 2) {
                            updateActiveName(kv.first.as<std::string>());
                        }
                        walkTreeRecursive(kv.second, node_root, node_name + "/" + kv.first.as<std::string>());
                        node_root.pop_back();
                        if (node_root.size() == 1) {
                            updateActiveName("");
                        }
                    } else {
                        std::string path = getYamlPath(kv.second.as<std::string>());
                        walkTree(path, node_root, node_name);
                    }
                }
            }
            break;
        }
        case YAML::NodeType::Null:
            break;
        default:
            EMPC_ERROR("Unsupported node Type at walkTreeR.");
            break;
    }
}

void ParserYaml::walkTree(std::string file)
{
    YAML::Node n;
    n = loadYaml(generatePath(file));
    parsing_file_.push(generatePath(file));
    std::vector<std::string> names = std::vector<std::string>();
    walkTreeRecursive(n, names, "");
    parsing_file_.pop();
}

void ParserYaml::walkTree(std::string file, std::vector<std::string>& root)
{
    YAML::Node n;
    n = loadYaml(generatePath(file));
    parsing_file_.push(generatePath(file));
    walkTreeRecursive(n, root, "");
    parsing_file_.pop();
}

void ParserYaml::walkTree(std::string file, std::vector<std::string>& root, std::string name)
{
    YAML::Node n;
    n = loadYaml(generatePath(file));
    parsing_file_.push(generatePath(file));
    walkTreeRecursive(n, root, name);
    parsing_file_.pop();
}

void ParserYaml::insertRegister(std::string key, std::string value)
{
    if (key.substr(0, 1) == "/") key = key.substr(1, key.size() - 1);
    auto inserted_it = params_.insert(std::pair<std::string, std::string>(key, value));
    // std::cout << "Inserted: " << key << " :: " << value << std::endl;
    if (not inserted_it.second)
        std::cout << "Skipping key '" << key << "' with value '" << value << "'. There already exists the register: ("
                  << inserted_it.first->first << "," << inserted_it.first->second << ")" << std::endl;
}

void ParserYaml::updateActiveName(std::string tag) { active_name_ = tag; }

const std::map<std::string, std::string>& ParserYaml::get_params() { return params_; }
}  // namespace eagle_mpc