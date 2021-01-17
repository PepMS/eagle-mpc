#ifndef MULTICOPTER_MPC_UTILS_PARAMS_SERVER_HPP
#define MULTICOPTER_MPC_UTILS_PARAMS_SERVER_HPP

#include "multicopter_mpc/utils/converter.hpp"

#include <vector>
#include <regex>
#include <map>
#include <exception>

namespace multicopter_mpc{

class MissingValueException : public std::runtime_error
{
public:
  MissingValueException(std::string msg) : std::runtime_error(msg) {}
};

class ParamsServer{
    std::map<std::string, std::string> _params;
public:
    ParamsServer(){
        _params = std::map<std::string, std::string>();
    }
    ParamsServer(std::map<std::string, std::string> params){
        _params = params;
    }
    ~ParamsServer(){
        //
    }

    void print(){
        for(auto it : _params)
            std::cout << it.first << "~~" << it.second << std::endl;
    }


    void addParam(std::string key, std::string value){
        _params.insert(std::pair<std::string, std::string>(key, value));
    }

//    template<typename T>
//    T getParam(std::string key, std::string def_value) const {
//        if(_params.find(key) != _params.end()){
//            return converter<T>::convert(_params.find(key)->second);
//        }else{
//            return converter<T>::convert(def_value);
//        }
//    }

    template<typename T>
    T getParam(std::string key) const {
        if(_params.find(key) != _params.end()){
            return converter<T>::convert(_params.find(key)->second);
        }else{
            throw MissingValueException("The following key: '" + key + "' has not been found in the parameters server.");
        }
    }

};

}

#endif
