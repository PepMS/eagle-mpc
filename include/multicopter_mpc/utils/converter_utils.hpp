#ifndef MULTICOPTER_MPC_UTILS_CONVERTER_UTILS_HPP
#define MULTICOPTER_MPC_UTILS_CONVERTER_UTILS_HPP

#include <regex>

namespace utils {
// Typically we want to convert from/to list-type structures. In order to be general
// we define a list type which is used throughout the converter. In this case this type
// is implemented with std::vector
template <typename A>
using list = std::vector<A>;
// template <typename A>
// class toString<A>{};
/** @Brief Splits a comma-separated string into its pieces
 * @param val is just the string of comma separated values
 * @return <b>{std::vector<std::string>}</b> vector whose i-th component is the i-th comma separated value
 */
std::vector<std::string> splitter(std::string val);
/** @Brief Returns all the substrings of @val that match @exp
 * @param val String to be matched
 * @param exp Regular expression
 * @return <b>{std::vector<std::string>}</b> Collection of matching substrings
 */
std::vector<std::string> getMatches(std::string val, std::regex exp);
/** @Brief Given a string representation of a matrix extracts the dimensions and the values
 * @param matrix is a string either of the form "[[N,M],[a1,a2,a3,...]]" or "[a1,a2,a3,...]"
 * @return <b>{std::array<std::string, 2>}</b> pair ([N,M],[a1,a2,a3,...]) or just ([a1,a2,a3...])
 */
std::array<std::string, 2> splitMatrixStringRepresentation(std::string matrix);
/** @Brief Splits a dictionary-like string of the form {k1:v1},{k2:v2},... It is tightly coupled with
 * splitMapStringRepresentation
 * @param val is just a dictionary-like string
 * @return <b>{std::vector<std::string>}</b> Collection of the strings of the form {k_i:v_i}
 */
std::vector<std::string> pairSplitter(std::string val);
/** @Brief Splits a dictionary-like string of the form [{k1:v1},{k2:v2},...]
 * @param str_map just a dictionary-like string
 * @return <b>{std::string}</b> String {k1:v1},{k2:v2},... (notice the removed brackets)
 */
std::string splitMapStringRepresentation(std::string str_map);
std::vector<std::string> parseList(std::string val);
}  // namespace utils
#endif