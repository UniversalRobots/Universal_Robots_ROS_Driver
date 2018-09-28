#pragma once

#include <ros/ros.h>
#include <ur_msgs/SetIO.h>
#include <ur_msgs/SetIORequest.h>
#include <ur_msgs/SetIOResponse.h>
#include <ur_msgs/SetPayload.h>
#include <ur_msgs/SetPayloadRequest.h>
#include <ur_msgs/SetPayloadResponse.h>
#include "ur_modern_driver/log.h"
#include "ur_modern_driver/ur/commander.h"

class IOService
{
private:
  ros::NodeHandle nh_;
  URCommander& commander_;
  ros::ServiceServer io_service_;
  ros::ServiceServer payload_service_;

  bool setIO(ur_msgs::SetIORequest& req, ur_msgs::SetIOResponse& resp)
  {
    LOG_INFO("setIO called with [%d, %d]", req.fun, req.pin);
    bool res = false;
    bool flag = req.state > 0.0 ? true : false;
    switch (req.fun)
    {
      case ur_msgs::SetIO::Request::FUN_SET_DIGITAL_OUT:
        res = commander_.setDigitalOut(req.pin, flag);
        break;
      case ur_msgs::SetIO::Request::FUN_SET_ANALOG_OUT:
        res = commander_.setAnalogOut(req.pin, req.state);
        break;
      case ur_msgs::SetIO::Request::FUN_SET_TOOL_VOLTAGE:
        res = commander_.setToolVoltage(static_cast<uint8_t>(req.state));
        break;
      case ur_msgs::SetIO::Request::FUN_SET_FLAG:
        res = commander_.setFlag(req.pin, flag);
        break;
      default:
        LOG_WARN("Invalid setIO function called (%d)", req.fun);
    }

    return (resp.success = res);
  }

  bool setPayload(ur_msgs::SetPayloadRequest& req, ur_msgs::SetPayloadResponse& resp)
  {
    LOG_INFO("setPayload called");
    // TODO check min and max payload?
    return (resp.success = commander_.setPayload(req.payload));
  }

public:
  IOService(URCommander& commander)
    : commander_(commander)
    , io_service_(nh_.advertiseService("ur_driver/set_io", &IOService::setIO, this))
    , payload_service_(nh_.advertiseService("ur_driver/set_payload", &IOService::setPayload, this))
  {
  }
};