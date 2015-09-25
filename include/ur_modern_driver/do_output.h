/*
 * do_output.h
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#ifndef UR_DO_OUTPUT_H_
#define UR_DO_OUTPUT_H_

#ifdef ROS_BUILD
#include <ros/ros.h>
#endif
#include <string>

void print_debug(std::string inp);
void print_info(std::string inp);
void print_warning(std::string inp);
void print_error(std::string inp);
void print_fatal(std::string inp);


#endif /* UR_DO_OUTPUT_H_ */
