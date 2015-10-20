/*
 * do_output.cpp
 *
 * Copyright 2015 Thomas Timm Andersen
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

#include "ur_modern_driver/do_output.h"

void print_debug(std::string inp) {
#ifdef ROS_BUILD
	ROS_DEBUG("%s", inp.c_str());
#else
	printf("DEBUG: %s\n", inp.c_str());
#endif
}
void print_info(std::string inp) {
#ifdef ROS_BUILD
	ROS_INFO("%s", inp.c_str());
#else
	printf("INFO: %s\n", inp.c_str());
#endif
}
void print_warning(std::string inp) {
#ifdef ROS_BUILD
	ROS_WARN("%s", inp.c_str());
#else
	printf("WARNING: %s\n", inp.c_str());
#endif
}
void print_error(std::string inp) {
#ifdef ROS_BUILD
	ROS_ERROR("%s", inp.c_str());
#else
	printf("ERROR: %s\n", inp.c_str());
#endif
}
void print_fatal(std::string inp) {
#ifdef ROS_BUILD
	ROS_FATAL("%s", inp.c_str());
	ros::shutdown();
#else
	printf("FATAL: %s\n", inp.c_str());
	exit(1);
#endif
}
