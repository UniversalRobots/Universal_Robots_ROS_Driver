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
	pri_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (pri_sockfd_ < 0) {
#ifdef ROS_BUILD
		ROS_FATAL("ERROR opening socket");
		ros::shutdown();
#else
		printf("ERROR opening socket");
		exit(1);
#endif
	}
	sec_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sec_sockfd_ < 0) {
#ifdef ROS_BUILD
		ROS_FATAL("ERROR opening socket");
		ros::shutdown();
#else
		printf("ERROR opening socket");
		exit(1);
#endif
	}
	server_ = gethostbyname(host.c_str());
	if (server_ == NULL) {
#ifdef ROS_BUILD
		ROS_FATAL("ERROR, no such host");
		ros::shutdown();
#else
		printf("ERROR, no such host\n");
		exit(1);
#endif
	}
	pri_serv_addr_.sin_family = AF_INET;
	sec_serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&pri_serv_addr_.sin_addr.s_addr, server_->h_length);
	bcopy((char *) server_->h_addr, (char *)&sec_serv_addr_.sin_addr.s_addr, server_->h_length);
	pri_serv_addr_.sin_port = htons(30001);
	sec_serv_addr_.sin_port = htons(30002);
	flag_ = 1;
	setsockopt(pri_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
			sizeof(int));
	setsockopt(pri_sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
			sizeof(int));
	setsockopt(sec_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
			sizeof(int));
	setsockopt(sec_sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
			sizeof(int));
	fcntl(sec_sockfd_, F_SETFL, O_NONBLOCK);
	connected_ = false;
	keepalive_ = false;
}

bool UrCommunication::start() {
	keepalive_ = true;
	uint8_t buf[512];
	unsigned int bytes_read;
	std::string cmd;
	bzero(buf, 512);
#ifdef ROS_BUILD
	ROS_DEBUG("Acquire firmware version: Connecting...");
#else
	printf("Acquire firmware version: Connecting...\n");
#endif
	if (connect(pri_sockfd_, (struct sockaddr *) &pri_serv_addr_,
			sizeof(pri_serv_addr_)) < 0) {
#ifdef ROS_BUILD
		ROS_FATAL("Error connecting");
		ros::shutdown();
		return false;
#else
		printf("Error connecting\n");
		return false;
#endif
	}
#ifdef ROS_BUILD
	ROS_DEBUG("Acquire firmware version: Got connection");
#else
	printf("Acquire firmware version: Got connection\n");
#endif
	bytes_read = read(pri_sockfd_, buf, 512);
	setsockopt(pri_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
			sizeof(int));
	robot_state_->unpack(buf, bytes_read);
	//wait for some traffic so the UR socket doesn't die in version 3.1.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
#ifdef ROS_BUILD
	ROS_DEBUG("Firmware version detected: %1.7f", robot_state_->getVersion());
#else
	printf("Firmware version detected: %f\n", robot_state_->getVersion());
#endif
	close(pri_sockfd_);

#ifdef ROS_BUILD
	ROS_DEBUG("Switching to secondary interface for masterboard data: Connecting...");
#else
	printf(
			"Switching to secondary interface for masterboard data: Connecting...\n"); // which generates less network traffic
#endif

	fd_set writefds;
	struct timeval timeout;

	connect(sec_sockfd_, (struct sockaddr *) &sec_serv_addr_,
			sizeof(sec_serv_addr_));
	FD_ZERO(&writefds);
	FD_SET(sec_sockfd_, &writefds);
	timeout.tv_sec = 10;
	timeout.tv_usec = 0;
	select(sec_sockfd_ + 1, NULL, &writefds, NULL, &timeout);
	unsigned int flag_len;
	getsockopt(sec_sockfd_, SOL_SOCKET, SO_ERROR, &flag_, &flag_len);
	if (flag_ < 0) {
#ifdef ROS_BUILD
		ROS_FATAL("Error connecting to secondary interface");
		ros::shutdown();
		return false;
#else
		printf("Error connecting to secondary interface\n");
		return false;
#endif
	}
#ifdef ROS_BUILD
	ROS_DEBUG("Secondary interface: Got connection");
#else
	printf("Secondary interface: Got connection\n");
#endif
	comThread_ = std::thread(&UrCommunication::run, this);
	return true;
}

void UrCommunication::halt() {
	keepalive_ = false;
	comThread_.join();
}

void UrCommunication::run() {
	uint8_t buf[2048];
	int bytes_read;
	bzero(buf, 2048);
	struct timeval timeout;
	fd_set readfds;
	FD_ZERO(&readfds);
	FD_SET(sec_sockfd_, &readfds);
	connected_ = true;
	while (keepalive_) {
		while (connected_ && keepalive_) {
			timeout.tv_sec = 0; //do this each loop as selects modifies timeout
			timeout.tv_usec = 500000; // timeout of 0.5 sec
			select(sec_sockfd_ + 1, &readfds, NULL, NULL, &timeout);
			bytes_read = read(sec_sockfd_, buf, 2048); // usually only up to 1295 bytes
			if (bytes_read > 0) {
				setsockopt(sec_sockfd_, IPPROTO_TCP, TCP_NODELAY,
						(char *) &flag_, sizeof(int));
				robot_state_->unpack(buf, bytes_read);
			} else {
				connected_ = false;
				close (sec_sockfd_);
			}
		}
		if (keepalive_) {
			//reconnect
#ifdef ROS_BUILD
			ROS_WARN("Secondary port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...");
#else
			printf(
					"Secondary port: No connection. Is controller crashed? Will try to reconnect in 10 seconds...\n");
#endif
			sec_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
			if (sec_sockfd_ < 0) {
#ifdef ROS_BUILD
				ROS_FATAL("ERROR opening secondary socket");
				ros::shutdown();
#else
				printf("ERROR opening secondary socket");
				exit(1);
#endif
			}
			flag_ = 1;
			setsockopt(sec_sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
					sizeof(int));
			setsockopt(sec_sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_,
					sizeof(int));
			fcntl(sec_sockfd_, F_SETFL, O_NONBLOCK);
			while (keepalive_ && !connected_) {
				std::this_thread::sleep_for(std::chrono::seconds(10));
				fd_set writefds;
				keepalive_ = true;

				connect(sec_sockfd_, (struct sockaddr *) &sec_serv_addr_,
						sizeof(sec_serv_addr_));
				FD_ZERO(&writefds);
				FD_SET(sec_sockfd_, &writefds);
				select(sec_sockfd_ + 1, NULL, &writefds, NULL, NULL);
				unsigned int flag_len;
				getsockopt(sec_sockfd_, SOL_SOCKET, SO_ERROR, &flag_,
						&flag_len);
				if (flag_ < 0) {
#ifdef ROS_BUILD
					ROS_ERROR("Error re-connecting to port 30002. Is controller started? Will try to reconnect in 10 seconds...");
#else
					printf(
							"Error re-connecting to port 30002. Is controller started? Will try to reconnect in 10 seconds...");
#endif
				} else {
					connected_ = true;
				}
			}
		}
	}

	//wait for some traffic so the UR socket doesn't die in version 3.1.
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	close(sec_sockfd_);
}

