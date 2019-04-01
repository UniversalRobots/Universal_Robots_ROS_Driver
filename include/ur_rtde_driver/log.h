/*
 * Copyright 2017, 2018 Simon Rasmussen (refactor)
 *
 * Copyright 2015, 2016 Thomas Timm Andersen (original version)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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