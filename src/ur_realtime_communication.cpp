/*
 * ur_realtime_communication.cpp
 *
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <thomas.timm.dk@gmail.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.   Thomas Timm Andersen
 * ----------------------------------------------------------------------------
 */

#include "ur_modern_driver/ur_realtime_communication.h"

UrRealtimeCommunication::UrRealtimeCommunication(
		std::condition_variable& msg_cond, std::string host,
		unsigned int safety_count_max) {
	robot_state_ = new RobotStateRT(msg_cond);
	bzero((char *) &serv_addr_, sizeof(serv_addr_));
	sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
	if (sockfd_ < 0) {
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
	serv_addr_.sin_family = AF_INET;
	bcopy((char *) server_->h_addr, (char *)&serv_addr_.sin_addr.s_addr, server_->h_length);
	serv_addr_.sin_port = htons(30003);
	flag_ = 1;
	setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_, sizeof(int));
	setsockopt(sockfd_, SOL_SOCKET, SO_REUSEADDR, (char *) &flag_, sizeof(int));
	connected_ = false;
	keepalive_ = false;
	safety_count_ = safety_count_max + 1;
	safety_count_max_ = safety_count_max;
}

void UrRealtimeCommunication::start() {
	keepalive_ = true;
#ifdef ROS_BUILD
	ROS_DEBUG("Realtime port: Connecting...");
#else
	printf("Realtime port: Connecting...\n");
#endif
	if (connect(sockfd_, (struct sockaddr *) &serv_addr_, sizeof(serv_addr_))
			< 0)
#ifdef ROS_BUILD
		ROS_FATAL("Error connecting to RT port 30003");
#else
		printf("Error connecting to RT port 30003\n");
#endif
	comThread_ = std::thread(&UrRealtimeCommunication::run, this);
}

void UrRealtimeCommunication::halt() {
	keepalive_ = false;
	comThread_.join();
}

void UrRealtimeCommunication::addCommandToQueue(std::string inp) {
	if (inp.back() != '\n') {
		inp.append("\n");
	}
	write(sockfd_, inp.c_str(), inp.length());
	/*command_string_lock_.lock();
	command_ += inp;
	command_string_lock_.unlock();*/

}

void UrRealtimeCommunication::setSpeed(double q0, double q1, double q2,
		double q3, double q4, double q5, double acc) {
	char cmd[1024];
	sprintf(cmd,
			"speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.02)\n",
			q0, q1, q2, q3, q4, q5, acc);
	addCommandToQueue((std::string) (cmd));
	if (q0 != 0. or q1 != 0. or q2 != 0. or q3 != 0. or q4 != 0. or q5 != 0.) {
		//If a joint speed is set, make sure we stop it again after some time if the user doesn't
		safety_count_ = 0;
	}
}

void UrRealtimeCommunication::run() {
	uint8_t buf[2048];
	bzero(buf, 2048);
#ifdef ROS_BUILD
	ROS_DEBUG("Realtime port: Got connection");
#else
	printf("Realtime port: Got connection\n");
#endif
	connected_ = true;
	while (keepalive_) {
		read(sockfd_, buf, 2048);
		setsockopt(sockfd_, IPPROTO_TCP, TCP_NODELAY, (char *) &flag_,
				sizeof(int));
		robot_state_->unpack(buf);
		/*command_string_lock_.lock();
		if (command_.length() != 0) {
			write(sockfd_, command_.c_str(), command_.length());
			command_ = "";
		}*/
		if (safety_count_ == safety_count_max_) {
			setSpeed(0., 0., 0., 0., 0., 0.);
			//write(sockfd_, command_.c_str(), command_.length());
			//command_ = "";
		}
		safety_count_ += 1;
		///command_string_lock_.unlock();

	}
	setSpeed(0., 0., 0., 0., 0., 0.);
	write(sockfd_, command_.c_str(), command_.length());
	close(sockfd_);
}

void UrRealtimeCommunication::setSafetyCountMax(uint inp) {
	safety_count_max_ = inp;
}

