#include <iostream>
#include <list>
#include <stack>
#include <vector>
#include <array>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "eagle_mpc/utils/converter_utils.hpp"

namespace utils {

std::vector<std::string> splitter(std::string val) {
  std::vector<std::string> cont = std::vector<std::string>();
  std::stringstream ss(val);
  std::string token;
  while (std::getline(ss, token, ',')) {
    cont.push_back(token);
  }
  return cont;
}

std::vector<std::string> getMatches(std::string val, std::regex exp) {
  std::smatch res;
  auto v = std::vector<std::string>();
  std::string::const_iterator searchStart(val.cbegin());
  while (std::regex_search(searchStart, val.cend(), res, exp)) {
    v.push_back(res[0]);
    searchStart = res.suffix().first;
  }
  return v;
}

std::array<std::string, 2> splitMatrixStringRepresentation(std::string matrix) {
  std::regex rgx("\\[\\[((?:[0-9]+,?)+)\\],((?:-?[0-9]*(?:\\.[0-9]+)?,?)+)\\]");
  std::regex rgxStatic("\\[((?:(?:-?[0-9]*)(?:\\.[0-9]+)?,?)+)\\]");
  std::smatch matches;
  std::smatch matchesStatic;
  std::array<std::string, 2> values = {{"[]", "[]"}};
  if (std::regex_match(matrix, matches, rgx)) {
    values[0] = "[" + matches[1].str() + "]";
    values[1] = "[" + matches[2].str() + "]";
  } else if (std::regex_match(matrix, matchesStatic, rgxStatic)) {
    values[1] = "[" + matchesStatic[1].str() + "]";
  } else {
    throw std::runtime_error(
        "Invalid string representation of a Matrix. Correct format is "
        "[([num,num],)?(num(,num)*)?]. String provided: " +
        matrix);
  }
  return values;
}

std::vector<std::string> pairSplitter(std::string val) {
  std::regex exp("\\{[^\\{:]:.*?\\}");
  return getMatches(val, exp);
}
std::string splitMapStringRepresentation(std::string str_map) {
  std::smatch mmatches;
  std::regex rgxM("\\[((?:(?:\\{[^\\{:]+:[^:\\}]+\\}),?)*)\\]");
  std::string result = "";
  if (std::regex_match(str_map, mmatches, rgxM)) {
    result = mmatches[1].str();
  } else {
    throw std::runtime_error(
        "Invalid string representation of a Map. Correct format is "
        "[({id:value})?(,{id:value})*]. String provided: " +
        str_map);
  }
  return result;
}

std::vector<std::string> parseList(std::string val) {
  std::stack<char> limiters;
  std::stack<std::string> word_stack;
  std::string current_word;
  std::vector<std::string> words;
  std::vector<char> chars(val.begin(), val.end());
  for (const char &current : chars) {
    if (current == '[') {
      limiters.push(current);
      word_stack.push(current_word);
      current_word = "";
    } else if (current == ']') {
      if (limiters.empty())
        throw std::runtime_error("Unmatched delimiter");
      if (limiters.top() == '[') {
        if (limiters.size() > 1) {
          if (word_stack.empty())
            word_stack.push("");
          current_word = word_stack.top() + "[" + current_word + "]";
          word_stack.pop();
        } else if (limiters.size() == 1 and current_word != "")
          words.push_back(current_word);
        else
          current_word += current;
        limiters.pop();
      } else
        throw std::runtime_error("Unmatched delimiter");
    } else if (current == '{') {
      limiters.push(current);
      word_stack.push(current_word);
      current_word = "";
    } else if (current == '}') {
      if (limiters.top() == '{') {
        if (limiters.size() > 1) {
          if (word_stack.empty())
            word_stack.push("");
          current_word = word_stack.top() + "{" + current_word + "}";
          word_stack.pop();
        } else if (limiters.size() == 1)
          words.push_back(current_word);
        else
          current_word += current;
        limiters.pop();
      } else
        throw std::runtime_error("Unmatched delimiter");
    } else if (current == ',') {
      if (limiters.size() == 1 and current_word != "") {
        words.push_back(current_word);
        current_word = "";
      } else if (limiters.size() > 1)
        current_word += current;
    } else {
      if (limiters.empty())
        throw std::runtime_error("Found non-delimited text");
      current_word += current;
    }
  }
  if (not limiters.empty())
    throw std::runtime_error("Unclosed delimiter [] or {}");
  return words;
}

} // namespace utils
