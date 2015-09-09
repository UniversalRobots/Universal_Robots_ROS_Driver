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
		std::vector<std::string> joint_names, unsigned int safety_count_max) {
	rt_interface_ = new UrRealtimeCommunication(msg_cond, host,
			safety_count_max);
	maximum_time_step_ = 2.;
	joint_names_ = joint_names;

}

std::vector<double> UrDriver::interp_cubic(double t, double T,
		std::vector<double> p0_pos, std::vector<double> p1_pos,
		std::vector<double> p0_vel, std::vector<double> p1_vel) {
	/*Returns positions of the joints at time 't' */
	std::vector<double> positions;
	for (unsigned int i = 0; i < p0_pos.size(); i++) {
		double a = p0_pos[i];
		double b = p0_vel[i];
		double c = (-3 * p0_pos[i] + 3 * p1_pos[i] - 2 * T * p0_vel[i]
				- T * p1_vel[i]) / pow(T, 2);
		double d = (2 * p0_pos[i] - 2 * p1_pos[i] + T * p0_vel[i]
				+ T * p1_vel[i]) / pow(T, 3);
		positions.push_back(a + b * t + c * pow(t, 2) + d * pow(t, 3));
	}
	return positions;
}

void UrDriver::addTraj(std::vector<double> inp_timestamps,
		std::vector<std::vector<double> > inp_positions,
		std::vector<std::vector<double> > inp_velocities) {

	std::vector<double> timestamps;
	std::vector<std::vector<double> > positions;
	std::string command_string = "def traj():\n";

	for (unsigned int i = 1; i < inp_timestamps.size(); i++) {
		timestamps.push_back(inp_timestamps[i - 1]);
		double dt = inp_timestamps[i] - inp_timestamps[i - 1];
		unsigned int steps = (unsigned int) ceil(dt / maximum_time_step_);
		double step_size = dt / steps;
		for (unsigned int j = 1; j < steps; j++) {
			timestamps.push_back(inp_timestamps[i - 1] + step_size * j);
		}
	}
	//make sure we come to a smooth stop
	while (timestamps.back() < inp_timestamps.back()) {
	 timestamps.push_back(timestamps.back() + 0.008);
	 }
	 timestamps.pop_back();

	unsigned int j = 0;
	for (unsigned int i = 0; i < timestamps.size(); i++) {
		while (inp_timestamps[j] <= timestamps[i]) {
			j += 1;
		}
		positions.push_back(
				UrDriver::interp_cubic(timestamps[i] - inp_timestamps[j-1], inp_timestamps[j] - inp_timestamps[j-1],
						inp_positions[j - 1], inp_positions[j],
						inp_velocities[j - 1], inp_velocities[j]));
	}

	timestamps.push_back(inp_timestamps[inp_timestamps.size() - 1]);
	positions.push_back(inp_positions[inp_positions.size() - 1]);
	for (unsigned int i = 1; i < timestamps.size(); i++) {
		char buf[128];
		sprintf(buf,
				"\tservoj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], t=%1.5f)\n",
				positions[i][0], positions[i][1], positions[i][2],
				positions[i][3], positions[i][4], positions[i][5],
				timestamps[i] - timestamps[i - 1]);
		command_string += buf;
	}
	command_string += "end\ntraj()\n";
	//printf("%s", command_string.c_str());
	rt_interface_->addCommandToQueue(command_string);

}

void UrDriver::stopTraj() {
	rt_interface_->addCommandToQueue("stopj(10)\n");
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

std::vector<std::string> UrDriver::getJointNames() {
	return joint_names_;
}

void UrDriver::setJointNames(std::vector<std::string> jn) {
	joint_names_ = jn;
}
