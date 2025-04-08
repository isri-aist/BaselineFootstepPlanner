#pragma once

#ifdef BFP_STANDALONE
#  include <cstdio>
#  include <iostream>
#  define BFP_ERROR_STREAM(x) std::cerr << x << "\n"
#  define BFP_WARN_STREAM(x) std::cerr << x << "\n"
#  define BFP_INFO_STREAM(x) std::cout << x << "\n"
#  define BFP_DEBUG(...) printf(__VA_ARGS__)
#  define BFP_INFO(...) printf(__VA_ARGS__)
#  define BFP_WARN(...) fprintf(stderr, __VA_ARGS__)
#  define BFP_INFO_THROTTLE(N, ...) printf(__VA_ARGS__)
#else
#  include <rclcpp/rclcpp.hpp>
#  define BFP_ERROR_STREAM(x) RCLCPP_ERROR(rclcpp::get_logger("BFP"), "%s", (x).c_str())
#  define BFP_WARN_STREAM(x) RCLCPP_WARN(rclcpp::get_logger("BFP"), "%s", (x).c_str())
#  define BFP_INFO_STREAM(x) RCLCPP_INFO(rclcpp::get_logger("BFP"), "%s", (x).c_str())
#  define BFP_DEBUG(...) printf(__VA_ARGS__)
#  define BFP_INFO(...) printf(__VA_ARGS__)
#  define BFP_WARN(...) fprintf(stderr, __VA_ARGS__)
#  define BFP_INFO_THROTTLE(N, ...) printf(__VA_ARGS__)
#endif
