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
#include <vector>
#include <math.h>

class UrDriver {
private:
	double maximum_time_step_;
	std::vector<std::string> joint_names_;
public:
	UrRealtimeCommunication* rt_interface_;

	UrDriver(std::condition_variable& msg_cond, std::string host,
			unsigned int safety_count_max = 12);
	void start();
	void halt();

	void setSpeed(double q0, double q1, double q2, double q3, double q4,
			double q5, double acc = 100.);
	void addTraj(std::vector<double> inp_timestamps,
			std::vector<std::vector<double> > positions,
			std::vector<std::vector<double> > velocities);
	void stopTraj();

	std::vector<double> interp_cubic(double t, double T,
			std::vector<double> p0_pos, std::vector<double> p1_pos,
			std::vector<double> p0_vel, std::vector<double> p1_vel);

	std::vector<std::string> getJointNames();
	void setJointNames(std::vector<std::string> jn);
	void setToolVoltage(unsigned int v);
	void setFlag(unsigned int n, bool b);
	void setDigitalOut(unsigned int n, bool b);
	void setAnalogOut(unsigned int n, double f);
	void setPayloaf(double m);

};

#endif /* UR_DRIVER_H_ */
