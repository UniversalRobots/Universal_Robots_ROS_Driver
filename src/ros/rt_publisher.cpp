#include "ur_modern_driver/ros/rt_publisher.h"

bool RTPublisher::publishJoints(RTShared& packet, Time& t)
{
  sensor_msgs::JointState joint_msg;
  joint_msg.header.stamp = t;

  joint_msg.name.assign(joint_names_.begin(), joint_names_.end());
  joint_msg.position.assign(packet.q_actual.begin(), packet.q_actual.end());
  joint_msg.velocity.assign(packet.qd_actual.begin(), packet.qd_actual.end());
  joint_msg.effort.assign(packet.i_actual.begin(), packet.i_actual.end());

  joint_pub_.publish(joint_msg);

  return true;
}

bool RTPublisher::publishWrench(RTShared& packet, Time& t)
{
  geometry_msgs::WrenchStamped wrench_msg;
  wrench_msg.header.stamp = t;
  wrench_msg.wrench.force.x = packet.tcp_force[0];
  wrench_msg.wrench.force.y = packet.tcp_force[1];
  wrench_msg.wrench.force.z = packet.tcp_force[2];
  wrench_msg.wrench.torque.x = packet.tcp_force[3];
  wrench_msg.wrench.torque.y = packet.tcp_force[4];
  wrench_msg.wrench.torque.z = packet.tcp_force[5];

  wrench_pub_.publish(wrench_msg);
  return true;
}

bool RTPublisher::publishTool(RTShared& packet, Time& t)
{
  geometry_msgs::TwistStamped tool_twist;
  tool_twist.header.stamp = t;
  tool_twist.header.frame_id = base_frame_;
  tool_twist.twist.linear.x = packet.tcp_speed_actual.position.x;
  tool_twist.twist.linear.y = packet.tcp_speed_actual.position.y;
  tool_twist.twist.linear.z = packet.tcp_speed_actual.position.z;
  tool_twist.twist.angular.x = packet.tcp_speed_actual.rotation.x;
  tool_twist.twist.angular.y = packet.tcp_speed_actual.rotation.y;
  tool_twist.twist.angular.z = packet.tcp_speed_actual.rotation.z;

  tool_vel_pub_.publish(tool_twist);
  return true;
}

bool RTPublisher::publishTransform(RTShared& packet, Time& t)
{
  auto tv = packet.tool_vector_actual;

  Transform transform;
  transform.setOrigin(Vector3(tv.position.x, tv.position.y, tv.position.z));

  // Create quaternion
  Quaternion quat;

  double angle = std::sqrt(std::pow(tv.rotation.x, 2) + std::pow(tv.rotation.y, 2) + std::pow(tv.rotation.z, 2));
  if (angle < 1e-16)
  {
    quat.setValue(0, 0, 0, 1);
  }
  else
  {
    quat.setRotation(Vector3(tv.rotation.x / angle, tv.rotation.y / angle, tv.rotation.z / angle), angle);
  }

  transform.setRotation(quat);

  transform_broadcaster_.sendTransform(StampedTransform(transform, t, base_frame_, tool_frame_));

  return true;
}

bool RTPublisher::publishTemperature(RTShared& packet, Time& t)
{
  size_t len = joint_names_.size();
  for (size_t i = 0; i < len; i++)
  {
    sensor_msgs::Temperature msg;
    msg.header.stamp = t;
    msg.header.frame_id = joint_names_[i];
    msg.temperature = packet.motor_temperatures[i];

    joint_temperature_pub_.publish(msg);
  }
  return true;
}

bool RTPublisher::publish(RTShared& packet)
{
  Time time = Time::now();
  bool res = true;
  if (!temp_only_)
  {
    res = publishJoints(packet, time) && publishWrench(packet, time) && publishTool(packet, time) &&
          publishTransform(packet, time);
  }

  return res && publishTemperature(packet, time);
}

bool RTPublisher::consume(RTState_V1_6__7& state)
{
  return publish(state);
}
bool RTPublisher::consume(RTState_V1_8& state)
{
  return publish(state);
}
bool RTPublisher::consume(RTState_V3_0__1& state)
{
  return publish(state);
}
bool RTPublisher::consume(RTState_V3_2__3& state)
{
  return publish(state);
}