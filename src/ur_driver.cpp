/*
 * ur_driver.cpp
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

#include "ur_modern_driver/ur_driver.h"

UrDriver::UrDriver(std::condition_variable& rt_msg_cond,
		std::condition_variable& msg_cond, std::string host,
		unsigned int reverse_port, double servoj_time,
		unsigned int safety_count_max, double max_time_step, double min_payload,
		double max_payload, double servoj_lookahead_time, double servoj_gain) :
		REVERSE_PORT_(reverse_port), maximum_time_step_(max_time_step), minimum_payload_(
				min_payload), maximum_payload_(max_payload), servoj_time_(
				servoj_time), servoj_lookahead_time_(servoj_lookahead_time), servoj_gain_(servoj_gain) {
	char buffer[256];
	struct sockaddr_in serv_addr;
	int n, flag;

	firmware_version_ = 0;
	reverse_connected_ = false;
	executing_traj_ = false;
	rt_interface_ = new UrRealtimeCommunication(rt_msg_cond, host,
			safety_count_max);
	new_sockfd_ = -1;
	sec_interface_ = new UrCommunication(msg_cond, host);

	incoming_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (incoming_sockfd_ < 0) {
		print_fatal("ERROR opening socket for reverse communication");
	}
	bzero((char *) &serv_addr, sizeof(serv_addr));

	serv_addr.sin_family = AF_INET;
	serv_addr.sin_addr.s_addr = INADDR_ANY;
	serv_addr.sin_port = htons(REVERSE_PORT_);
	flag = 1;
	setsockopt(incoming_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag,
			sizeof(int));
	setsockopt(incoming_sockfd_, SOL_SOCKET, SO_REUSEADDR, &flag, sizeof(int));
	if (bind(incoming_sockfd_, (struct sockaddr *) &serv_addr,
			sizeof(serv_addr)) < 0) {
		print_fatal("ERROR on binding socket for reverse communication");
	}
	listen(incoming_sockfd_, 5);
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

bool UrDriver::doTraj(std::vector<double> inp_timestamps,
		std::vector<std::vector<double> > inp_positions,
		std::vector<std::vector<double> > inp_velocities) {
	std::chrono::high_resolution_clock::time_point t0, t;
	std::vector<double> positions;
	unsigned int j;

	if (!UrDriver::uploadProg()) {
		return false;
	}
	executing_traj_ = true;
	t0 = std::chrono::high_resolution_clock::now();
	t = t0;
	j = 0;
	while ((inp_timestamps[inp_timestamps.size() - 1]
			>= std::chrono::duration_cast<std::chrono::duration<double>>(t - t0).count())
			and executing_traj_) {
		while (inp_timestamps[j]
				<= std::chrono::duration_cast<std::chrono::duration<double>>(
						t - t0).count() && j < inp_timestamps.size() - 1) {
			j += 1;
		}
		positions = UrDriver::interp_cubic(
				std::chrono::duration_cast<std::chrono::duration<double>>(
						t - t0).count() - inp_timestamps[j - 1],
				inp_timestamps[j] - inp_timestamps[j - 1], inp_positions[j - 1],
				inp_positions[j], inp_velocities[j - 1], inp_velocities[j]);
		UrDriver::servoj(positions);

		// oversample with 4 * sample_time
		std::this_thread::sleep_for(
				std::chrono::milliseconds((int) ((servoj_time_ * 1000) / 4.)));
		t = std::chrono::high_resolution_clock::now();
	}
	executing_traj_ = false;
	//Signal robot to stop driverProg()
	UrDriver::closeServo(positions);
	return true;
}

void UrDriver::servoj(std::vector<double> positions, int keepalive) {
	if (!reverse_connected_) {
		print_error(
				"UrDriver::servoj called without a reverse connection present. Keepalive: "
						+ std::to_string(keepalive));
		return;
	}
	unsigned int bytes_written;
	int tmp;
	unsigned char buf[28];
	for (int i = 0; i < 6; i++) {
		tmp = htonl((int) (positions[i] * MULT_JOINTSTATE_));
		buf[i * 4] = tmp & 0xff;
		buf[i * 4 + 1] = (tmp >> 8) & 0xff;
		buf[i * 4 + 2] = (tmp >> 16) & 0xff;
		buf[i * 4 + 3] = (tmp >> 24) & 0xff;
	}
	tmp = htonl((int) keepalive);
	buf[6 * 4] = tmp & 0xff;
	buf[6 * 4 + 1] = (tmp >> 8) & 0xff;
	buf[6 * 4 + 2] = (tmp >> 16) & 0xff;
	buf[6 * 4 + 3] = (tmp >> 24) & 0xff;
	bytes_written = write(new_sockfd_, buf, 28);
}

void UrDriver::stopTraj() {
	executing_traj_ = false;
	rt_interface_->addCommandToQueue("stopj(10)\n");
}

bool UrDriver::uploadProg() {
	std::string cmd_str;
	char buf[128];
	cmd_str = "def driverProg():\n";

	sprintf(buf, "\tMULT_jointstate = %i\n", MULT_JOINTSTATE_);
	cmd_str += buf;

	cmd_str += "\tSERVO_IDLE = 0\n";
	cmd_str += "\tSERVO_RUNNING = 1\n";
	cmd_str += "\tcmd_servo_state = SERVO_IDLE\n";
	cmd_str += "\tcmd_servo_q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]\n";
	cmd_str += "\tdef set_servo_setpoint(q):\n";
	cmd_str += "\t\tenter_critical\n";
	cmd_str += "\t\tcmd_servo_state = SERVO_RUNNING\n";
	cmd_str += "\t\tcmd_servo_q = q\n";
	cmd_str += "\t\texit_critical\n";
	cmd_str += "\tend\n";
	cmd_str += "\tthread servoThread():\n";
	cmd_str += "\t\tstate = SERVO_IDLE\n";
	cmd_str += "\t\twhile True:\n";
	cmd_str += "\t\t\tenter_critical\n";
	cmd_str += "\t\t\tq = cmd_servo_q\n";
	cmd_str += "\t\t\tdo_brake = False\n";
	cmd_str += "\t\t\tif (state == SERVO_RUNNING) and ";
	cmd_str += "(cmd_servo_state == SERVO_IDLE):\n";
	cmd_str += "\t\t\t\tdo_brake = True\n";
	cmd_str += "\t\t\tend\n";
	cmd_str += "\t\t\tstate = cmd_servo_state\n";
	cmd_str += "\t\t\tcmd_servo_state = SERVO_IDLE\n";
	cmd_str += "\t\t\texit_critical\n";
	cmd_str += "\t\t\tif do_brake:\n";
	cmd_str += "\t\t\t\tstopj(1.0)\n";
	cmd_str += "\t\t\t\tsync()\n";
	cmd_str += "\t\t\telif state == SERVO_RUNNING:\n";

	if (sec_interface_->robot_state_->getVersion() >= 3.1)
		sprintf(buf, "\t\t\t\tservoj(q, t=%.4f, lookahead_time=%.4f, gain=%.0f)\n",
				servoj_time_, servoj_lookahead_time_, servoj_gain_);
	else
		sprintf(buf, "\t\t\t\tservoj(q, t=%.4f)\n", servoj_time_);
	cmd_str += buf;

	cmd_str += "\t\t\telse:\n";
	cmd_str += "\t\t\t\tsync()\n";
	cmd_str += "\t\t\tend\n";
	cmd_str += "\t\tend\n";
	cmd_str += "\tend\n";

	sprintf(buf, "\tsocket_open(\"%s\", %i)\n", ip_addr_.c_str(),
			REVERSE_PORT_);
	cmd_str += buf;

	cmd_str += "\tthread_servo = run servoThread()\n";
	cmd_str += "\tkeepalive = 1\n";
	cmd_str += "\twhile keepalive > 0:\n";
	cmd_str += "\t\tparams_mult = socket_read_binary_integer(6+1)\n";
	cmd_str += "\t\tif params_mult[0] > 0:\n";
	cmd_str += "\t\t\tq = [params_mult[1] / MULT_jointstate, ";
	cmd_str += "params_mult[2] / MULT_jointstate, ";
	cmd_str += "params_mult[3] / MULT_jointstate, ";
	cmd_str += "params_mult[4] / MULT_jointstate, ";
	cmd_str += "params_mult[5] / MULT_jointstate, ";
	cmd_str += "params_mult[6] / MULT_jointstate]\n";
	cmd_str += "\t\t\tkeepalive = params_mult[7]\n";
	cmd_str += "\t\t\tset_servo_setpoint(q)\n";
	cmd_str += "\t\tend\n";
	cmd_str += "\tend\n";
	cmd_str += "\tsleep(.1)\n";
	cmd_str += "\tsocket_close()\n";
	cmd_str += "\tkill thread_servo\n";
	cmd_str += "end\n";

	rt_interface_->addCommandToQueue(cmd_str);
	return UrDriver::openServo();
}

bool UrDriver::openServo() {
	struct sockaddr_in cli_addr;
	socklen_t clilen;
	clilen = sizeof(cli_addr);
	new_sockfd_ = accept(incoming_sockfd_, (struct sockaddr *) &cli_addr,
			&clilen);
	if (new_sockfd_ < 0) {
		print_fatal("ERROR on accepting reverse communication");
		return false;
	}
	reverse_connected_ = true;
	return true;
}
void UrDriver::closeServo(std::vector<double> positions) {
	if (positions.size() != 6)
		UrDriver::servoj(rt_interface_->robot_state_->getQActual(), 0);
	else
		UrDriver::servoj(positions, 0);

	reverse_connected_ = false;
	close(new_sockfd_);
}

bool UrDriver::start() {
	if (!sec_interface_->start())
		return false;
	firmware_version_ = sec_interface_->robot_state_->getVersion();
	rt_interface_->robot_state_->setVersion(firmware_version_);
	if (!rt_interface_->start())
		return false;
	ip_addr_ = rt_interface_->getLocalIp();
	print_debug(
			"Listening on " + ip_addr_ + ":" + std::to_string(REVERSE_PORT_)
					+ "\n");
	return true;

}

void UrDriver::halt() {
	if (executing_traj_) {
		UrDriver::stopTraj();
	}
	sec_interface_->halt();
	rt_interface_->halt();
	close(incoming_sockfd_);
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

void UrDriver::setToolVoltage(unsigned int v) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_tool_voltage(%d)\nend\n", v);
	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);
}
void UrDriver::setFlag(unsigned int n, bool b) {
	char buf[256];
	sprintf(buf, "sec setOut():\n\tset_flag(%d, %s)\nend\n", n,
			b ? "True" : "False");
	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);
}
void UrDriver::setDigitalOut(unsigned int n, bool b) {
	char buf[256];
	if (firmware_version_ < 2) {
		sprintf(buf, "sec setOut():\n\tset_digital_out(%d, %s)\nend\n", n,
				b ? "True" : "False");
    } else if (n > 15) {
        sprintf(buf,
                "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n",
                n - 16, b ? "True" : "False");
	} else if (n > 7) {
        sprintf(buf, "sec setOut():\n\tset_configurable_digital_out(%d, %s)\nend\n",
				n - 8, b ? "True" : "False");

	} else {
		sprintf(buf, "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n",
				n, b ? "True" : "False");

	}
	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);

}
void UrDriver::setAnalogOut(unsigned int n, double f) {
	char buf[256];
	if (firmware_version_ < 2) {
		sprintf(buf, "sec setOut():\n\tset_analog_out(%d, %1.4f)\nend\n", n, f);
	} else {
		sprintf(buf, "sec setOut():\n\tset_standard_analog_out(%d, %1.4f)\nend\n", n, f);
	}

	rt_interface_->addCommandToQueue(buf);
	print_debug(buf);
}

bool UrDriver::setPayload(double m) {
	if ((m < maximum_payload_) && (m > minimum_payload_)) {
		char buf[256];
		sprintf(buf, "sec setOut():\n\tset_payload(%1.3f)\nend\n", m);
		rt_interface_->addCommandToQueue(buf);
		print_debug(buf);
		return true;
	} else
		return false;
}

void UrDriver::setMinPayload(double m) {
	if (m > 0) {
		minimum_payload_ = m;
	} else {
		minimum_payload_ = 0;
	}

}
void UrDriver::setMaxPayload(double m) {
	maximum_payload_ = m;
}
void UrDriver::setServojTime(double t) {
	if (t > 0.008) {
		servoj_time_ = t;
	} else {
		servoj_time_ = 0.008;
	}
}
void UrDriver::setServojLookahead(double t){
	if (t > 0.03) {
		if (t < 0.2) {
			servoj_lookahead_time_ = t;
		} else {
			servoj_lookahead_time_ = 0.2;
		}
	} else {
		servoj_lookahead_time_ = 0.03;
	}
}
void UrDriver::setServojGain(double g){
	if (g > 100) {
			if (g < 2000) {
				servoj_gain_ = g;
			} else {
				servoj_gain_ = 2000;
			}
		} else {
			servoj_gain_ = 100;
		}
}
