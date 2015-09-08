/*
 * ur_driver.cpp
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#include "ur_modern_driver/ur_driver.h"

UrDriver::UrDriver(std::condition_variable& msg_cond, std::string host,
		unsigned int safety_count_max) {
	rt_interface_ = new UrRealtimeCommunication(msg_cond, host,
			safety_count_max);

}

void UrDriver::start() {
	rt_interface_->start();
}

void UrDriver::halt() {
	rt_interface_->halt();
}

void UrDriver::setSpeed(double q0, double q1, double q2, double q3, double q4,
		double q5, double acc) {
	rt_interface_->setSpeed(q0, q1, q2, q3, q4, q5, acc);
}

