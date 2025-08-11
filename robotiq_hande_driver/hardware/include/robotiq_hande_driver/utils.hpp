#ifndef ROBOTIQ_HANDE_DRIVER__UTILS_HPP_
#define ROBOTIQ_HANDE_DRIVER__UTILS_HPP_

namespace robotiq_hande_driver {

namespace color {
// Reset
constexpr const char* RESET = "\033[0m";

// Regular colors
constexpr const char* BLACK = "\033[30m";
constexpr const char* RED = "\033[31m";
constexpr const char* GREEN = "\033[32m";
constexpr const char* YELLOW = "\033[33m";
constexpr const char* BLUE = "\033[34m";
constexpr const char* MAGENTA = "\033[35m";
constexpr const char* CYAN = "\033[36m";
constexpr const char* WHITE = "\033[37m";

// Bold/bright variants
constexpr const char* BBLACK = "\033[1;30m";
constexpr const char* BRED = "\033[1;31m";
constexpr const char* BGREEN = "\033[1;32m";
constexpr const char* BYELLOW = "\033[1;33m";
constexpr const char* BBLUE = "\033[1;34m";
constexpr const char* BMAGENTA = "\033[1;35m";
constexpr const char* BCYAN = "\033[1;36m";
constexpr const char* BWHITE = "\033[1;37m";

// Background colors (optional)
constexpr const char* BG_BLACK = "\033[40m";
constexpr const char* BG_RED = "\033[41m";
constexpr const char* BG_GREEN = "\033[42m";
constexpr const char* BG_YELLOW = "\033[43m";
constexpr const char* BG_BLUE = "\033[44m";
constexpr const char* BG_MAGENTA = "\033[45m";
constexpr const char* BG_CYAN = "\033[46m";
constexpr const char* BG_WHITE = "\033[47m";
}  // namespace color

}  // namespace robotiq_hande_driver
#endif  // ROBOTIQ_HANDE_DRIVER__UTILS_HPP_
