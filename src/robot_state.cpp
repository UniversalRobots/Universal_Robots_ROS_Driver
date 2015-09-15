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
	val_lock_.lock();
	switch (robot_message_type) {
	case robotMessageType::ROBOT_MESSAGE_VERSION:
		version_msg_.timestamp = timestamp;
		version_msg_.source = source;
		version_msg_.robot_message_type = robot_message_type;
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
		memcpy(&version_msg_.build_date, &buf[offset],
				sizeof(char) * len - offset);

		break;
	default:
		break;
	}
	val_lock_.unlock();

}
void RobotState::unpackRobotState(uint8_t * buf, unsigned int offset,
		uint32_t len) {

}

double RobotState::getVersion() {
	double ver;
	val_lock_.lock();
	ver = version_msg_.major_version + 0.1 * version_msg_.minor_version + .000001*version_msg_.svn_revision;
	val_lock_.unlock();
	return ver;

}

