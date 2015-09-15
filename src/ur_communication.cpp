/*
 * ur_communication.cpp
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#include "ur_modern_driver/ur_communication.h"

UrCommunication::UrCommunication(std::condition_variable& msg_cond,
		std::string host) {
	robot_state_ = new RobotState(msg_cond);
	bzero((char *) &pri_serv_addr_, sizeof(pri_serv_addr_));
	bzero((char *) &sec_serv_addr_, sizeof(sec_serv_addr_));
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) {
		printf("ERROR opening socket");
		exit(0);
	}
	server_ = gethostbyname(host.c_str());
	if (server_ == NULL) {
		printf("ERROR, no such host\n");
		exit(0);
	}
	pri_serv_addr_.sin_family = AF_INET;
	sec_serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&pri_serv_addr_.sin_addr.s_addr, server_->h_length);
	bcopy((char *) server_->h_addr, (char *)&sec_serv_addr_.sin_addr.s_addr, server_->h_length);
	pri_serv_addr_.sin_port = htons(30001);
	sec_serv_addr_.sin_port = htons(30002);
	flag_ = 1;
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_, sizeof(int));
	connected_ = false;
	keepalive_ = false;
}

void UrCommunication::start() {
	keepalive_ = true;
	uint8_t buf[512];
	unsigned int bytes_read;
	std::string cmd;
	bzero(buf, 512);

	printf("Acquire firmware version: Connecting...\n");
	if (connect(sockfd_, (struct sockaddr *) &pri_serv_addr_,
			sizeof(pri_serv_addr_)) < 0) {
		printf("Error connecting");
		exit(1);
	}
	printf("Acquire firmware version: Got connection\n");
	bytes_read = read(sockfd_, buf, 512);
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
	robot_state_->unpack(buf, bytes_read);
	//wait for some traffic so the UR socket doesn't die in version 3.1.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	printf("Firmware version: %f\n\n", robot_state_->getVersion());
	close(sockfd_);

	printf("Switching to secondary interface for masterboard data: Connecting...\n"); // which generates less network traffic
		if (connect(sockfd_, (struct sockaddr *) &sec_serv_addr_,
				sizeof(sec_serv_addr_)) < 0) {
			printf("Error connecting");
			exit(1);
		}
		printf("Secondary interface: Got connection\n");
	comThread_ = std::thread(&UrCommunication::run, this);
}

void UrCommunication::halt() {
	keepalive_ = false;
	comThread_.join();
}

void UrCommunication::addCommandToQueue(std::string inp) {
	if (inp.back() != '\n') {
		inp.append("\n");
	}
	command_string_lock_.lock();
	command_ += inp;
	command_string_lock_.unlock();
}

void UrCommunication::run() {
	uint8_t buf[2048];
	unsigned int bytes_read;
	bzero(buf, 2048);
	printf("Got connection\n");
	connected_ = true;
	while (keepalive_) {
		bytes_read = read(sockfd_, buf, 2048); // usually only up to 1295 bytes
		setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
				sizeof(int));
		robot_state_->unpack(buf, bytes_read);
		command_string_lock_.lock();
		if (command_.length() != 0) {
			write(sockfd_, command_.c_str(), command_.length());
			command_ = "";
		}
		command_string_lock_.unlock();

	}
	close(sockfd_);
}

