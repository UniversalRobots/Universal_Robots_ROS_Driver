/*
 * ur_driver
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#ifndef UR_DRIVER_H_
#define UR_DRIVER_H_

#include <mutex>
#include <condition_variable>
#include "ur_realtime_communication.h"

class UrDriver {
public:
	UrRealtimeCommunication* rt_interface_;

	UrDriver(std::condition_variable& msg_cond, std::string host,
			uint safety_count_max = 12);
	void start();
	void halt();
	void setSpeed(double q0, double q1, double q2, double q3, double q4,
			double q5, double acc = 100.);

};

#endif /* UR_DRIVER_H_ */
