/*
 * do_output.cpp
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#include "ur_modern_driver/do_output.h"

void print_debug(std::string inp) {
#ifdef ROS_BUILD
	ROS_DEBUG(inp.c_str());
#else
	printf("DEBUG: %s\n", inp.c_str());
#endif
}
void print_info(std::string inp) {
#ifdef ROS_BUILD
	ROS_INFO(inp.c_str());
#else
	printf("INFO: %s\n", inp.c_str());
#endif
}
void print_warning(std::string inp) {
#ifdef ROS_BUILD
	ROS_WARN(inp.c_str());
#else
	printf("WARNING: %s\n", inp.c_str());
#endif
}
void print_error(std::string inp) {
#ifdef ROS_BUILD
	ROS_ERROR(inp.c_str());
#else
	printf("ERROR: %s\n", inp.c_str());
#endif
}
void print_fatal(std::string inp) {
#ifdef ROS_BUILD
	ROS_FATAL(inp.c_str());
	ros::shutdown();
#else
	printf("FATAL: %s\n", inp.c_str());
	exit(1);
#endif
}
