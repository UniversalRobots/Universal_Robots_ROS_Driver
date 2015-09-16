/*
 * robot_state.cpp
 *
 *  Created on: Sep 10, 2015
 *      Author: ttan
 */

#include "ur_modern_driver/robot_state.h"

RobotState::RobotState(std::condition_variable& msg_cond) {
	version_msg_.major_version = 0;
	version_msg_.minor_version = 0;
	new_data_available_ = false;
	pMsg_cond_ = &msg_cond;
}

int RobotState::unpack(uint8_t* buf, unsigned int buf_length) {
	/* Returns missing bytes to unpack a message, or 0 if all data was parsed */
	unsigned int offset = 0;
	while (buf_length > offset) {
		uint32_t len;
		unsigned char message_type;
		memcpy(&len, &buf[offset], sizeof(len));
		if (len + offset > buf_length) {
			return (len + offset - buf_length);
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
	return 0;
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
		uint32_t length;
		int8_t package_type;
		memcpy(&length, &buf[offset], sizeof(length));
		offset += sizeof(length);
		memcpy(&package_type, &buf[offset], sizeof(package_type));
		offset += sizeof(package_type);
		switch (package_type) {
		case packageType::MASTERBOARD_DATA:
			val_lock_.lock();
			RobotState::unpackRobotStateMasterboard(buf, offset);
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
	memcpy(&version_msg_.major_version, &buf[offset],
			sizeof(version_msg_.major_version));
	offset += sizeof(version_msg_.major_version);
	memcpy(&version_msg_.minor_version, &buf[offset],
			sizeof(version_msg_.minor_version));
	offset += sizeof(version_msg_.minor_version);
	memcpy(&version_msg_.svn_revision, &buf[offset],
			sizeof(version_msg_.svn_revision));
	offset += sizeof(version_msg_.svn_revision);
	memcpy(&version_msg_.build_date, &buf[offset], sizeof(char) * len - offset);
}

void RobotState::unpackRobotStateMasterboard(uint8_t * buf,
		unsigned int offset) {
	if (RobotState::getVersion() < 3.0) {
		int16_t digital_input_bits, digital_output_bits;
		memcpy(&digital_input_bits, &buf[offset], sizeof(digital_input_bits));
		offset += sizeof(digital_input_bits);
		memcpy(&digital_output_bits, &buf[offset], sizeof(digital_output_bits));
		offset += sizeof(digital_output_bits);
		mb_data_.digitalInputBits = digital_input_bits;
		mb_data_.digitalOutputBits = digital_output_bits;
	} else {
		memcpy(&mb_data_.digitalInputBits, &buf[offset],
				sizeof(mb_data_.digitalInputBits));
		offset += sizeof(mb_data_.digitalInputBits);
		memcpy(&mb_data_.digitalOutputBits, &buf[offset],
				sizeof(mb_data_.digitalOutputBits));
		offset += sizeof(mb_data_.digitalOutputBits);
	}
	memcpy(&mb_data_.analogInputRange0, &buf[offset],
			sizeof(mb_data_.analogInputRange0));
	offset += sizeof(mb_data_.analogInputRange0);
	memcpy(&mb_data_.analogInputRange1, &buf[offset],
			sizeof(mb_data_.analogInputRange1));
	offset += sizeof(mb_data_.analogInputRange1);
	memcpy(&mb_data_.analogInput0, &buf[offset], sizeof(mb_data_.analogInput0));
	offset += sizeof(mb_data_.analogInput0);
	memcpy(&mb_data_.analogInput1, &buf[offset], sizeof(mb_data_.analogInput1));
	offset += sizeof(mb_data_.analogInput1);
	memcpy(&mb_data_.analogOutputDomain0, &buf[offset],
			sizeof(mb_data_.analogOutputDomain0));
	offset += sizeof(mb_data_.analogOutputDomain0);
	memcpy(&mb_data_.analogOutputDomain1, &buf[offset],
			sizeof(mb_data_.analogOutputDomain1));
	offset += sizeof(mb_data_.analogOutputDomain1);
	memcpy(&mb_data_.analogOutput0, &buf[offset],
			sizeof(mb_data_.analogOutput0));
	offset += sizeof(mb_data_.analogOutput0);
	memcpy(&mb_data_.analogOutput1, &buf[offset],
			sizeof(mb_data_.analogOutput1));
	offset += sizeof(mb_data_.analogOutput1);

	memcpy(&mb_data_.masterBoardTemperature, &buf[offset],
			sizeof(mb_data_.masterBoardTemperature));
	offset += sizeof(mb_data_.masterBoardTemperature);
	memcpy(&mb_data_.robotVoltage48V, &buf[offset],
			sizeof(mb_data_.robotVoltage48V));
	offset += sizeof(mb_data_.robotVoltage48V);
	memcpy(&mb_data_.robotCurrent, &buf[offset], sizeof(mb_data_.robotCurrent));
	offset += sizeof(mb_data_.robotCurrent);
	memcpy(&mb_data_.masterIOCurrent, &buf[offset],
			sizeof(mb_data_.masterIOCurrent));
	offset += sizeof(mb_data_.masterIOCurrent);

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
		memcpy(&mb_data_.euromapOutputBits, &buf[offset],
				sizeof(mb_data_.euromapOutputBits));
		offset += sizeof(mb_data_.euromapOutputBits);
		if (RobotState::getVersion() < 3.0) {
			int16_t euromap_voltage, euromap_current;
			memcpy(&euromap_voltage, &buf[offset], sizeof(euromap_voltage));
			offset += sizeof(euromap_voltage);
			memcpy(&euromap_current, &buf[offset], sizeof(euromap_current));
			offset += sizeof(euromap_current);
			mb_data_.euromapVoltage = euromap_voltage;
			mb_data_.euromapCurrent = euromap_current;
		} else {
			memcpy(&mb_data_.euromapVoltage, &buf[offset],
					sizeof(mb_data_.euromapVoltage));
			offset += sizeof(mb_data_.euromapVoltage);
			memcpy(&mb_data_.euromapCurrent, &buf[offset],
					sizeof(mb_data_.euromapCurrent));
			offset += sizeof(mb_data_.euromapCurrent);
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
