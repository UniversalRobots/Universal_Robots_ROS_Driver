/*
 * robotStateRT.h
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

#ifndef ROBOT_STATE_RT_H_
#define ROBOT_STATE_RT_H_

#include <inttypes.h>
#include <vector>
#include <stdlib.h>
#include <string.h>
#include <mutex>
#include <netinet/in.h>
#include <condition_variable>

class RobotStateRT {
private:
	double version_; //protocol version

	double time_; //Time elapsed since the controller was started
	std::vector<double> q_target_; //Target joint positions
	std::vector<double> qd_target_; //Target joint velocities
	std::vector<double> qdd_target_; //Target joint accelerations
	std::vector<double> i_target_; //Target joint currents
	std::vector<double> m_target_; //Target joint moments (torques)
	std::vector<double> q_actual_; //Actual joint positions
	std::vector<double> qd_actual_; //Actual joint velocities
	std::vector<double> i_actual_; //Actual joint currents
	std::vector<double> i_control_; //Joint control currents
	std::vector<double> tool_vector_actual_; //Actual Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
	std::vector<double> tcp_speed_actual_; //Actual speed of the tool given in Cartesian coordinates
	std::vector<double> tcp_force_; //Generalised forces in the TC
	std::vector<double> tool_vector_target_; //Target Cartesian coordinates of the tool: (x,y,z,rx,ry,rz), where rx, ry and rz is a rotation vector representation of the tool orientation
	std::vector<double> tcp_speed_target_; //Target speed of the tool given in Cartesian coordinates
	std::vector<bool> digital_input_bits_; //Current state of the digital inputs. NOTE: these are bits encoded as int64_t, e.g. a value of 5 corresponds to bit 0 and bit 2 set high
	std::vector<double> motor_temperatures_; //Temperature of each joint in degrees celsius
	double controller_timer_; //Controller realtime thread execution time
	double robot_mode_; //Robot mode
	std::vector<double> joint_modes_; //Joint control modes
	double safety_mode_; //Safety mode
	std::vector<double> tool_accelerometer_values_; //Tool x,y and z accelerometer values (software version 1.7)
	double speed_scaling_; //Speed scaling of the trajectory limiter
	double linear_momentum_norm_; //Norm of Cartesian linear momentum
	double v_main_; //Masterboard: Main voltage
	double v_robot_; //Matorborad: Robot voltage (48V)
	double i_robot_; //Masterboard: Robot current
	std::vector<double> v_actual_; //Actual joint voltages

	std::mutex val_lock_; // Locks the variables while unpack parses data;

	std::condition_variable* pMsg_cond_; //Signals that new vars are available
	bool data_published_; //to avoid spurious wakes
	bool controller_updated_; //to avoid spurious wakes

	std::vector<double> unpackVector(uint8_t * buf, int start_index,
			int nr_of_vals);
	std::vector<bool> unpackDigitalInputBits(int64_t data);
	double ntohd(uint64_t nf);

public:
	RobotStateRT(std::condition_variable& msg_cond);
	~RobotStateRT();
	double getVersion();
	double getTime();
	std::vector<double> getQTarget();
	std::vector<double> getQdTarget();
	std::vector<double> getQddTarget();
	std::vector<double> getITarget();
	std::vector<double> getMTarget();
	std::vector<double> getQActual();
	std::vector<double> getQdActual();
	std::vector<double> getIActual();
	std::vector<double> getIControl();
	std::vector<double> getToolVectorActual();
	std::vector<double> getTcpSpeedActual();
	std::vector<double> getTcpForce();
	std::vector<double> getToolVectorTarget();
	std::vector<double> getTcpSpeedTarget();
	std::vector<bool> getDigitalInputBits();
	std::vector<double> getMotorTemperatures();
	double getControllerTimer();
	double getRobotMode();
	std::vector<double> getJointModes();
	double getSafety_mode();
	std::vector<double> getToolAccelerometerValues();
	double getSpeedScaling();
	double getLinearMomentumNorm();
	double getVMain();
	double getVRobot();
	double getIRobot();

	void setVersion(double ver);

	void setDataPublished();
	bool getDataPublished();
	bool getControllerUpdated();
	void setControllerUpdated();
	std::vector<double> getVActual();
	void unpack(uint8_t * buf);
};

#endif /* ROBOT_STATE_RT_H_ */
