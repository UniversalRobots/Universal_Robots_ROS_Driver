/*
 * ur_realtime_communication.h
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

#ifndef UR_REALTIME_COMMUNICATION_H_
#define UR_REALTIME_COMMUNICATION_H_

#include "robot_state_RT.h"
#include "do_output.h"
#include <vector>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <iostream>
#include <unistd.h>
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/types.h>

class UrRealtimeCommunication {
private:
	unsigned int safety_count_max_;
	int sockfd_;
	struct sockaddr_in serv_addr_;
	struct hostent *server_;
	std::string local_ip_;
	bool keepalive_;
	std::thread comThread_;
	int flag_;
	std::recursive_mutex command_string_lock_;
	std::string command_;
	unsigned int safety_count_;
	void run();


public:
	bool connected_;
	RobotStateRT* robot_state_;

	UrRealtimeCommunication(std::condition_variable& msg_cond, std::string host,
			unsigned int safety_count_max = 12);
	bool start();
	void halt();
	void setSpeed(double q0, double q1, double q2, double q3, double q4,
			double q5, double acc = 100.);
	void addCommandToQueue(std::string inp);
	void setSafetyCountMax(uint inp);
	std::string getLocalIp();

};

#endif /* UR_REALTIME_COMMUNICATION_H_ */
