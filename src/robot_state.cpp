/*
 * robot_state.cpp
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

#include "ur_modern_driver/robot_state.h"

RobotState::RobotState(std::condition_variable& msg_cond) {
	version_msg_.major_version = 0;
	version_msg_.minor_version = 0;
	new_data_available_ = false;
	pMsg_cond_ = &msg_cond;
	RobotState::setDisconnected();
	robot_mode_running_ = robotStateTypeV30::ROBOT_MODE_RUNNING;
}
double RobotState::ntohd(uint64_t nf) {
	double x;
	nf = be64toh(nf);
	memcpy(&x, &nf, sizeof(x));
	return x;
}
void RobotState::unpack(uint8_t* buf, unsigned int buf_length) {
	/* Returns missing bytes to unpack a message, or 0 if all data was parsed */
	unsigned int offset = 0;
	while (buf_length > offset) {
		int len;
		unsigned char message_type;
		memcpy(&len, &buf[offset], sizeof(len));
		len = ntohl(len);
		if (len + offset > buf_length) {
			return;
		}
		memcpy(&message_type, &buf[offset + sizeof(len)], sizeof(message_type));
		switch (message_type) {
		case messageType::ROBOT_MESSAGE:
			RobotState::unpackRobotMessage(buf, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
			break;
		case messageType::ROBOT_STATE:
			RobotState::unpackRobotState(buf, offset, len); //'len' is inclusive the 5 bytes from messageSize and messageType
			break;
		case messageType::PROGRAM_STATE_MESSAGE:
			//Don't do anything atm...
		default:
			break;
		}
		offset += len;

	}
	return;
}

void RobotState::unpackRobotMessage(uint8_t * buf, unsigned int offset,
		uint32_t len) {
	offset += 5;
	uint64_t timestamp;
	int8_t source, robot_message_type;
	memcpy(&timestamp, &buf[offset], sizeof(timestamp));
	offset += sizeof(timestamp);
	memcpy(&source, &buf[offset], sizeof(source));
	offset += sizeof(source);
	memcpy(&robot_message_type, &buf[offset], sizeof(robot_message_type));
	offset += sizeof(robot_message_type);
	switch (robot_message_type) {
	case robotMessageType::ROBOT_MESSAGE_VERSION:
		val_lock_.lock();
		version_msg_.timestamp = timestamp;
		version_msg_.source = source;
		version_msg_.robot_message_type = robot_message_type;
		RobotState::unpackRobotMessageVersion(buf, offset, len);
		val_lock_.unlock();
		break;
	default:
		break;
	}

}

void RobotState::unpackRobotState(uint8_t * buf, unsigned int offset,
		uint32_t len) {
	offset += 5;
	while (offset < len) {
		int32_t length;
		uint8_t package_type;
		memcpy(&length, &buf[offset], sizeof(length));
		length = ntohl(length);
		memcpy(&package_type, &buf[offset + sizeof(length)],
				sizeof(package_type));
		switch (package_type) {
		case packageType::ROBOT_MODE_DATA:
			val_lock_.lock();
			RobotState::unpackRobotMode(buf, offset + 5);
			val_lock_.unlock();
			break;

		case packageType::MASTERBOARD_DATA:
			val_lock_.lock();
			RobotState::unpackRobotStateMasterboard(buf, offset + 5);
			val_lock_.unlock();
			break;
		default:
			break;
		}
		offset += length;
	}
	new_data_available_ = true;
	pMsg_cond_->notify_all();

}

void RobotState::unpackRobotMessageVersion(uint8_t * buf, unsigned int offset,
		uint32_t len) {
	memcpy(&version_msg_.project_name_size, &buf[offset],
			sizeof(version_msg_.project_name_size));
	offset += sizeof(version_msg_.project_name_size);
	memcpy(&version_msg_.project_name, &buf[offset],
			sizeof(char) * version_msg_.project_name_size);
	offset += version_msg_.project_name_size;
	version_msg_.project_name[version_msg_.project_name_size] = '\0';
	memcpy(&version_msg_.major_version, &buf[offset],
			sizeof(version_msg_.major_version));
	offset += sizeof(version_msg_.major_version);
	memcpy(&version_msg_.minor_version, &buf[offset],
			sizeof(version_msg_.minor_version));
	offset += sizeof(version_msg_.minor_version);
	memcpy(&version_msg_.svn_revision, &buf[offset],
			sizeof(version_msg_.svn_revision));
	offset += sizeof(version_msg_.svn_revision);
	version_msg_.svn_revision = ntohl(version_msg_.svn_revision);
	memcpy(&version_msg_.build_date, &buf[offset], sizeof(char) * len - offset);
	version_msg_.build_date[len - offset] = '\0';
	if (version_msg_.major_version < 2) {
		robot_mode_running_ = robotStateTypeV18::ROBOT_RUNNING_MODE;
	}
}

void RobotState::unpackRobotMode(uint8_t * buf, unsigned int offset) {
	memcpy(&robot_mode_.timestamp, &buf[offset], sizeof(robot_mode_.timestamp));
	offset += sizeof(robot_mode_.timestamp);
	uint8_t tmp;
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0)
		robot_mode_.isRobotConnected = true;
	else
		robot_mode_.isRobotConnected = false;
	offset += sizeof(tmp);
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0)
		robot_mode_.isRealRobotEnabled = true;
	else
		robot_mode_.isRealRobotEnabled = false;
	offset += sizeof(tmp);
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	//printf("PowerOnRobot: %d\n", tmp);
	if (tmp > 0)
		robot_mode_.isPowerOnRobot = true;
	else
		robot_mode_.isPowerOnRobot = false;
	offset += sizeof(tmp);
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0)
		robot_mode_.isEmergencyStopped = true;
	else
		robot_mode_.isEmergencyStopped = false;
	offset += sizeof(tmp);
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0)
		robot_mode_.isProtectiveStopped = true;
	else
		robot_mode_.isProtectiveStopped = false;
	offset += sizeof(tmp);
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0)
		robot_mode_.isProgramRunning = true;
	else
		robot_mode_.isProgramRunning = false;
	offset += sizeof(tmp);
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0)
		robot_mode_.isProgramPaused = true;
	else
		robot_mode_.isProgramPaused = false;
	offset += sizeof(tmp);
	memcpy(&robot_mode_.robotMode, &buf[offset], sizeof(robot_mode_.robotMode));
	offset += sizeof(robot_mode_.robotMode);
	uint64_t temp;
	if (RobotState::getVersion() > 2.) {
		memcpy(&robot_mode_.controlMode, &buf[offset],
				sizeof(robot_mode_.controlMode));
		offset += sizeof(robot_mode_.controlMode);
		memcpy(&temp, &buf[offset], sizeof(temp));
		offset += sizeof(temp);
		robot_mode_.targetSpeedFraction = RobotState::ntohd(temp);
	}
	memcpy(&temp, &buf[offset], sizeof(temp));
	offset += sizeof(temp);
	robot_mode_.speedScaling = RobotState::ntohd(temp);
}

void RobotState::unpackRobotStateMasterboard(uint8_t * buf,
		unsigned int offset) {
	if (RobotState::getVersion() < 3.0) {
		int16_t digital_input_bits, digital_output_bits;
		memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
		offset += sizeof(digital_input_bits);
		memcpy(&digital_output_bits, &buf[offset], sizeof(digital_output_bits));
		offset += sizeof(digital_output_bits);
		mb_data_.digitalInputBits = ntohs(digital_input_bits);
		mb_data_.digitalOutputBits = ntohs(digital_output_bits);
	} else {
		memcpy(&mb_data_.digitalInputBits, &buf[offset],
				sizeof(mb_data_.digitalInputBits));
		offset += sizeof(mb_data_.digitalInputBits);
		mb_data_.digitalInputBits = ntohl(mb_data_.digitalInputBits);
		memcpy(&mb_data_.digitalOutputBits, &buf[offset],
				sizeof(mb_data_.digitalOutputBits));
		offset += sizeof(mb_data_.digitalOutputBits);
		mb_data_.digitalOutputBits = ntohl(mb_data_.digitalOutputBits);
	}

	memcpy(&mb_data_.analogInputRange0, &buf[offset],
			sizeof(mb_data_.analogInputRange0));
	offset += sizeof(mb_data_.analogInputRange0);
	memcpy(&mb_data_.analogInputRange1, &buf[offset],
			sizeof(mb_data_.analogInputRange1));
	offset += sizeof(mb_data_.analogInputRange1);
	uint64_t temp;
	memcpy(&temp, &buf[offset], sizeof(temp));
	offset += sizeof(temp);
	mb_data_.analogInput0 = RobotState::ntohd(temp);
	memcpy(&temp, &buf[offset], sizeof(temp));
	offset += sizeof(temp);
	mb_data_.analogInput1 = RobotState::ntohd(temp);
	memcpy(&mb_data_.analogOutputDomain0, &buf[offset],
			sizeof(mb_data_.analogOutputDomain0));
	offset += sizeof(mb_data_.analogOutputDomain0);
	memcpy(&mb_data_.analogOutputDomain1, &buf[offset],
			sizeof(mb_data_.analogOutputDomain1));
	offset += sizeof(mb_data_.analogOutputDomain1);
	memcpy(&temp, &buf[offset], sizeof(temp));
	offset += sizeof(temp);
	mb_data_.analogOutput0 = RobotState::ntohd(temp);
	memcpy(&temp, &buf[offset], sizeof(temp));
	offset += sizeof(temp);
	mb_data_.analogOutput1 = RobotState::ntohd(temp);

	memcpy(&mb_data_.masterBoardTemperature, &buf[offset],
			sizeof(mb_data_.masterBoardTemperature));
	offset += sizeof(mb_data_.masterBoardTemperature);
	mb_data_.masterBoardTemperature = ntohl(mb_data_.masterBoardTemperature);
	memcpy(&mb_data_.robotVoltage48V, &buf[offset],
			sizeof(mb_data_.robotVoltage48V));
	offset += sizeof(mb_data_.robotVoltage48V);
	mb_data_.robotVoltage48V = ntohl(mb_data_.robotVoltage48V);
	memcpy(&mb_data_.robotCurrent, &buf[offset], sizeof(mb_data_.robotCurrent));
	offset += sizeof(mb_data_.robotCurrent);
	mb_data_.robotCurrent = ntohl(mb_data_.robotCurrent);
	memcpy(&mb_data_.masterIOCurrent, &buf[offset],
			sizeof(mb_data_.masterIOCurrent));
	offset += sizeof(mb_data_.masterIOCurrent);
	mb_data_.masterIOCurrent = ntohl(mb_data_.masterIOCurrent);

	memcpy(&mb_data_.safetyMode, &buf[offset], sizeof(mb_data_.safetyMode));
	offset += sizeof(mb_data_.safetyMode);
	memcpy(&mb_data_.masterOnOffState, &buf[offset],
			sizeof(mb_data_.masterOnOffState));
	offset += sizeof(mb_data_.masterOnOffState);

	memcpy(&mb_data_.euromap67InterfaceInstalled, &buf[offset],
			sizeof(mb_data_.euromap67InterfaceInstalled));
	offset += sizeof(mb_data_.euromap67InterfaceInstalled);
	if (mb_data_.euromap67InterfaceInstalled != 0) {
		memcpy(&mb_data_.euromapInputBits, &buf[offset],
				sizeof(mb_data_.euromapInputBits));
		offset += sizeof(mb_data_.euromapInputBits);
		mb_data_.euromapInputBits = ntohl(mb_data_.euromapInputBits);
		memcpy(&mb_data_.euromapOutputBits, &buf[offset],
				sizeof(mb_data_.euromapOutputBits));
		offset += sizeof(mb_data_.euromapOutputBits);
		mb_data_.euromapOutputBits = ntohl(mb_data_.euromapOutputBits);
		if (RobotState::getVersion() < 3.0) {
			int16_t euromap_voltage, euromap_current;
			memcpy(&euromap_voltage, &buf[offset], sizeof(euromap_voltage));
			offset += sizeof(euromap_voltage);
			memcpy(&euromap_current, &buf[offset], sizeof(euromap_current));
			offset += sizeof(euromap_current);
			mb_data_.euromapVoltage = ntohs(euromap_voltage);
			mb_data_.euromapCurrent = ntohs(euromap_current);
		} else {
			memcpy(&mb_data_.euromapVoltage, &buf[offset],
					sizeof(mb_data_.euromapVoltage));
			offset += sizeof(mb_data_.euromapVoltage);
			mb_data_.euromapVoltage = ntohl(mb_data_.euromapVoltage);
			memcpy(&mb_data_.euromapCurrent, &buf[offset],
					sizeof(mb_data_.euromapCurrent));
			offset += sizeof(mb_data_.euromapCurrent);
			mb_data_.euromapCurrent = ntohl(mb_data_.euromapCurrent);
		}

	}
}

double RobotState::getVersion() {
	double ver;
	val_lock_.lock();
	ver = version_msg_.major_version + 0.1 * version_msg_.minor_version
			+ .0000001 * version_msg_.svn_revision;
	val_lock_.unlock();
	return ver;

}

void RobotState::finishedReading() {
	new_data_available_ = false;
}

bool RobotState::getNewDataAvailable() {
	return new_data_available_;
}

int RobotState::getDigitalInputBits() {
	return mb_data_.digitalInputBits;
}
int RobotState::getDigitalOutputBits() {
	return mb_data_.digitalOutputBits;
}
double RobotState::getAnalogInput0() {
	return mb_data_.analogInput0;
}
double RobotState::getAnalogInput1() {
	return mb_data_.analogInput1;
}
double RobotState::getAnalogOutput0() {
	return mb_data_.analogOutput0;

}
double RobotState::getAnalogOutput1() {
	return mb_data_.analogOutput1;
}
bool RobotState::isRobotConnected() {
	return robot_mode_.isRobotConnected;
}
bool RobotState::isRealRobotEnabled() {
	return robot_mode_.isRealRobotEnabled;
}
bool RobotState::isPowerOnRobot() {
	return robot_mode_.isPowerOnRobot;
}
bool RobotState::isEmergencyStopped() {
	return robot_mode_.isEmergencyStopped;
}
bool RobotState::isProtectiveStopped() {
	return robot_mode_.isProtectiveStopped;
}
bool RobotState::isProgramRunning() {
	return robot_mode_.isProgramRunning;
}
bool RobotState::isProgramPaused() {
	return robot_mode_.isProgramPaused;
}
unsigned char RobotState::getRobotMode() {
	return robot_mode_.robotMode;
}
bool RobotState::isReady() {
	if (robot_mode_.robotMode == robot_mode_running_) {
		return true;
	}
	return false;
}

void RobotState::setDisconnected() {
	robot_mode_.isRobotConnected = false;
	robot_mode_.isRealRobotEnabled = false;
	robot_mode_.isPowerOnRobot = false;
}
