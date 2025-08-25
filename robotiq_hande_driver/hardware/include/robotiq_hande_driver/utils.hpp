#ifndef ROBOTIQ_HANDE_DRIVER__UTILS_HPP_
#define ROBOTIQ_HANDE_DRIVER__UTILS_HPP_

#include <algorithm>
#include <cctype>
#include <string>

namespace robotiq_hande_driver {

// Symbols for formatting the CLI output.
namespace color {

// Reset
constexpr const char* RESET = "\033[0m";

// Regular colors
constexpr const char* RED = "\033[31m";
constexpr const char* GREEN = "\033[32m";
constexpr const char* YELLOW = "\033[33m";
constexpr const char* CYAN = "\033[36m";

// Bold/bright variants
constexpr const char* BRED = "\033[1;31m";
constexpr const char* BGREEN = "\033[1;32m";
constexpr const char* BYELLOW = "\033[1;33m";
constexpr const char* BCYAN = "\033[1;36m";

}  // namespace color

std::string str_to_lower(const std::string& str) {
    std::string s(str);
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);
    return s;
}

}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__UTILS_HPP_
