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

namespace yaml_parser {
class ParserYAML {
    struct ParamsInitSensor{
        std::string type_;
        std::string name_;
        std::string plugin_;
        YAML::Node n_;
    };
    struct ParamsInitProcessor{
        std::string type_;
        std::string name_;
        std::string name_assoc_sensor_;
        std::string plugin_;
        YAML::Node n_;
    };
    struct SubscriberManager{
        std::string package_;
        std::string subscriber_;
        std::string topic_;
        std::string sensor_name_;
        YAML::Node n_;
    };
    struct PublisherManager{
        std::string package_;
        std::string subscriber_;
        std::string topic_;
        std::string period_;
        YAML::Node n_;
    };
    std::map<std::string, std::string> params_;
    std::string active_name_;
    std::vector<ParamsInitSensor> paramsSens_;
    std::vector<ParamsInitProcessor> paramsProc_;
    std::stack<std::string> parsing_file_;
    std::string file_;
    std::string path_root_;
    std::vector<SubscriberManager> subscriber_managers_;
    std::vector<PublisherManager> publisher_managers_;
    YAML::Node problem;
    std::string generatePath(std::string);
    YAML::Node loadYAML(std::string);
    void insert_register(std::string, std::string);
public:
    ParserYAML(std::string _file, std::string _path_root = "",
               bool _freely_parse = false);
    ~ParserYAML()
    {
        //
    }
    void parse_freely();
    std::map<std::string, std::string> getParams();

  private:
    void walkTree(std::string _file);
    void walkTree(std::string _file, std::vector<std::string>& _tags);
    void walkTree(std::string _file, std::vector<std::string>& _tags, std::string _hdr);
/** @Brief Recursively walks the YAML tree while filling a map with the values parsed from the file
 * @param YAML node to be parsed
 * @param tags represents the path from the root of the YAML tree to the current node
 * @param hdr is the name of the current YAML node
 */
    void walkTreeR(YAML::Node _n, std::vector<std::string>& _tags, std::string _hdr);
    void updateActiveName(std::string _tag);
/** @Brief Parse the sensors, processors, callbacks and files elements of the YAML file. We assume that these elements
    are defined at the top level of the YAML file.
 * @param file is the path to the YAML file */
    void parseFirstLevel(std::string _file);
    std::string tagsToString(std::vector<std::string>& _tags);
    void parse();
};
}  // namespace yaml_parser
#endif
