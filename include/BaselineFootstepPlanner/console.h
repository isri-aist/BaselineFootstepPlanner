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
#  include <ros/console.h>
#  define BFP_ERROR_STREAM ROS_ERROR_STREAM
#  define BFP_WARN_STREAM ROS_WARN_STREAM
#  define BFP_INFO_STREAM ROS_INFO_STREAM
#  define BFP_DEBUG ROS_DEBUG
#  define BFP_INFO ROS_INFO
#  define BFP_WARN ROS_WARN
#  define BFP_INFO_THROTTLE ROS_INFO_THROTTLE
#endif
