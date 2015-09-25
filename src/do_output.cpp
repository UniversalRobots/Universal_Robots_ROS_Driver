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

#include "ur_modern_driver/du_output.h"

void print_debug(std::string inp) {
#ifdef ROS_BUILD
	ROS_DEBUG(inp);
#else
	printf("DEBUG: %s\n", inp.c_str());
#endif
}
void print_info(std::string inp) {
#ifdef ROS_BUILD
	ROS_INFO(inp);
#else
	printf("INFO: %s\n", inp.c_str());
#endif
}
void print_warning(std::string inp) {
#ifdef ROS_BUILD
	ROS_WARN(inp);
#else
	printf("WARNING: %s\n", inp.c_str());
#endif
}
void print_error(std::string inp) {
#ifdef ROS_BUILD
	ROS_ERROR(inp);
#else
	printf("ERROR: %s\n", inp.c_str());
#endif
}
void print_fatal(std::string inp) {
#ifdef ROS_BUILD
	ROS_FATAL(inp);
	ros::shutdown();
#else
	printf("FATAL: %s\n", inp.c_str());
	exit(1);
#endif
}
