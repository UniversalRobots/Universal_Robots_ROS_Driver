#pragma once
#include <inttypes.h>

#ifdef ROS_BUILD
#include <ros/ros.h>

#define LOG_DEBUG ROS_DEBUG
#define LOG_WARN ROS_WARN
#define LOG_INFO ROS_INFO
#define LOG_ERROR ROS_ERROR
#define LOG_FATAL ROS_FATAL

#else

#define LOG_DEBUG(format, ...) printf("[DEBUG]: " format "\n", ##__VA_ARGS__)
#define LOG_WARN(format, ...) printf("[WARNING]: " format "\n", ##__VA_ARGS__)
#define LOG_INFO(format, ...) printf("[INFO]: " format "\n", ##__VA_ARGS__)
#define LOG_ERROR(format, ...) printf("[ERROR]: " format "\n", ##__VA_ARGS__)
#define LOG_FATAL(format, ...) printf("[FATAL]: " format "\n", ##__VA_ARGS__)

#endif