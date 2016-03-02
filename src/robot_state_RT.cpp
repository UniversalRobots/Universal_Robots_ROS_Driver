/*
 * robotStateRT.cpp
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

#include "ur_modern_driver/robot_state_RT.h"

RobotStateRT::RobotStateRT(std::condition_variable& msg_cond) {
	version_ = 0.0;
	time_ = 0.0;
	q_target_.assign(6, 0.0);
	qd_target_.assign(6, 0.0);
	qdd_target_.assign(6, 0.0);
	i_target_.assign(6, 0.0);
	m_target_.assign(6, 0.0);
	q_actual_.assign(6, 0.0);
	qd_actual_.assign(6, 0.0);
	i_actual_.assign(6, 0.0);
	i_control_.assign(6, 0.0);
	tool_vector_actual_.assign(6, 0.0);
	tcp_speed_actual_.assign(6, 0.0);
	tcp_force_.assign(6, 0.0);
	tool_vector_target_.assign(6, 0.0);
	tcp_speed_target_.assign(6, 0.0);
	digital_input_bits_.assign(64, false);
	motor_temperatures_.assign(6, 0.0);
	controller_timer_ = 0.0;
	robot_mode_ = 0.0;
	joint_modes_.assign(6, 0.0);
	safety_mode_ = 0.0;
	tool_accelerometer_values_.assign(3, 0.0);
	speed_scaling_ = 0.0;
	linear_momentum_norm_ = 0.0;
	v_main_ = 0.0;
	v_robot_ = 0.0;
	i_robot_ = 0.0;
	v_actual_.assign(6, 0.0);
	data_published_ = false;
	controller_updated_ = false;
	pMsg_cond_ = &msg_cond;
}

RobotStateRT::~RobotStateRT() {
	/* Make sure nobody is waiting after this thread is destroyed */
	data_published_ = true;
	controller_updated_ = true;
	pMsg_cond_->notify_all();
}

void RobotStateRT::setDataPublished() {
	data_published_ = false;
}
bool RobotStateRT::getDataPublished() {
	return data_published_;
}

void RobotStateRT::setControllerUpdated() {
	controller_updated_ = false;
}
bool RobotStateRT::getControllerUpdated() {
	return controller_updated_;
}

double RobotStateRT::ntohd(uint64_t nf) {
	double x;
	nf = be64toh(nf);
	memcpy(&x, &nf, sizeof(x));
	return x;
}

std::vector<double> RobotStateRT::unpackVector(uint8_t * buf, int start_index,
		int nr_of_vals) {
	uint64_t q;
	std::vector<double> ret;
	for (int i = 0; i < nr_of_vals; i++) {
		memcpy(&q, &buf[start_index + i * sizeof(q)], sizeof(q));
		ret.push_back(ntohd(q));
	}
	return ret;
}

std::vector<bool> RobotStateRT::unpackDigitalInputBits(int64_t data) {
	std::vector<bool> ret;
	for (int i = 0; i < 64; i++) {
		ret.push_back((data & (1 << i)) >> i);
	}
	return ret;
}

void RobotStateRT::setVersion(double ver) {
	val_lock_.lock();
	version_ = ver;
	val_lock_.unlock();
}

double RobotStateRT::getVersion() {
	double ret;
	val_lock_.lock();
	ret = version_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getTime() {
	double ret;
	val_lock_.lock();
	ret = time_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = q_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQdTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qd_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQddTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qdd_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getITarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getMTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = m_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = q_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getQdActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = qd_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getIActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getIControl() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = i_control_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getToolVectorActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_vector_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getTcpSpeedActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_speed_actual_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getTcpForce() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_force_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getToolVectorTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_vector_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getTcpSpeedTarget() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tcp_speed_target_;
	val_lock_.unlock();
	return ret;
}
std::vector<bool> RobotStateRT::getDigitalInputBits() {
	std::vector<bool> ret;
	val_lock_.lock();
	ret = digital_input_bits_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getMotorTemperatures() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = motor_temperatures_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getControllerTimer() {
	double ret;
	val_lock_.lock();
	ret = controller_timer_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getRobotMode() {
	double ret;
	val_lock_.lock();
	ret = robot_mode_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getJointModes() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = joint_modes_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getSafety_mode() {
	double ret;
	val_lock_.lock();
	ret = safety_mode_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getToolAccelerometerValues() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = tool_accelerometer_values_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getSpeedScaling() {
	double ret;
	val_lock_.lock();
	ret = speed_scaling_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getLinearMomentumNorm() {
	double ret;
	val_lock_.lock();
	ret = linear_momentum_norm_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getVMain() {
	double ret;
	val_lock_.lock();
	ret = v_main_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getVRobot() {
	double ret;
	val_lock_.lock();
	ret = v_robot_;
	val_lock_.unlock();
	return ret;
}
double RobotStateRT::getIRobot() {
	double ret;
	val_lock_.lock();
	ret = i_robot_;
	val_lock_.unlock();
	return ret;
}
std::vector<double> RobotStateRT::getVActual() {
	std::vector<double> ret;
	val_lock_.lock();
	ret = v_actual_;
	val_lock_.unlock();
	return ret;
}
void RobotStateRT::unpack(uint8_t * buf) {
	int64_t digital_input_bits;
	uint64_t unpack_to;
	uint16_t offset = 0;
	val_lock_.lock();
	int len;
	memcpy(&len, &buf[offset], sizeof(len));

	offset += sizeof(len);
	len = ntohl(len);

	//Check the correct message length is received
	bool len_good = true;
	if (version_ >= 1.6 && version_ < 1.7) { //v1.6
		if (len != 756)
			len_good = false;
	} else if (version_ >= 1.7 && version_ < 1.8) { //v1.7
		if (len != 764)
			len_good = false;
	} else if (version_ >= 1.8 && version_ < 1.9) { //v1.8
		if (len != 812)
			len_good = false;
	} else if (version_ >= 3.0 && version_ < 3.2) { //v3.0 & v3.1
		if (len != 1044)
			len_good = false;
	} else if (version_ >= 3.2 && version_ < 3.3) { //v3.2
		if (len != 1060)
			len_good = false;
	}

	if (!len_good) {
		printf("Wrong length of message on RT interface: %i\n", len);
		val_lock_.unlock();
		return;
	}

	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	time_ = RobotStateRT::ntohd(unpack_to);
	offset += sizeof(double);
	q_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	qd_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	qdd_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	i_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	m_target_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	q_actual_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	qd_actual_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	i_actual_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	if (version_ <= 1.9) {
		if (version_ > 1.6)
			tool_accelerometer_values_ = unpackVector(buf, offset, 3);
		offset += sizeof(double) * (3 + 15);
		tcp_force_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tool_vector_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tcp_speed_actual_ = unpackVector(buf, offset, 6);
	} else {
		i_control_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tool_vector_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tcp_speed_actual_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tcp_force_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tool_vector_target_ = unpackVector(buf, offset, 6);
		offset += sizeof(double) * 6;
		tcp_speed_target_ = unpackVector(buf, offset, 6);
	}
	offset += sizeof(double) * 6;

	memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
	digital_input_bits_ = unpackDigitalInputBits(be64toh(digital_input_bits));
	offset += sizeof(double);
	motor_temperatures_ = unpackVector(buf, offset, 6);
	offset += sizeof(double) * 6;
	memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
	controller_timer_ = ntohd(unpack_to);
	if (version_ > 1.6) {
		offset += sizeof(double) * 2;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		robot_mode_ = ntohd(unpack_to);
		if (version_ > 1.7) {
			offset += sizeof(double);
			joint_modes_ = unpackVector(buf, offset, 6);
		}
	}
	if (version_ > 1.8) {
		offset += sizeof(double) * 6;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		safety_mode_ = ntohd(unpack_to);
		offset += sizeof(double);
		tool_accelerometer_values_ = unpackVector(buf, offset, 3);
		offset += sizeof(double) * 3;
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		speed_scaling_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		linear_momentum_norm_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		v_main_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		v_robot_ = ntohd(unpack_to);
		offset += sizeof(double);
		memcpy(&unpack_to, &buf[offset], sizeof(unpack_to));
		i_robot_ = ntohd(unpack_to);
		offset += sizeof(double);
		v_actual_ = unpackVector(buf, offset, 6);
	}
	val_lock_.unlock();
	controller_updated_ = true;
	data_published_ = true;
	pMsg_cond_->notify_all();

}

