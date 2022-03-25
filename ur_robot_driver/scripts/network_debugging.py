#!/usr/bin/env python3

import argparse
import socket
import subprocess

import rospy


def get_own_ip(remote_addr: str, remote_port: int = 30001):
    """Get IP address own local interface connecting to a remote interface

    :param remote_addr: IP address (or hostname) of remote machine. This is required in order to
    select the correct local interface.
    :type remote_addr: str
    :param remote_port: Port that should be used to establish a test connection to the remote
    :type remote_port: int
    """
    socket_connection = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Connect to robot's primary interface on port 30001
    socket_connection.connect((remote_addr, remote_port))
    return socket_connection.getsockname()[0]


class ConnectionDebugger:
    """Small utility to debug a connection to a UR robot"""

    def __init__(self):
        rospy.init_node("connection_debugger")

        self.remote_user = rospy.get_param("~remote_user", "root")
        self.remote_password = rospy.get_param("~remote_password", "easybot")

        self.robot_ip = self._get_hw_param("robot_ip")
        self.reverse_ip = self._get_hw_param("reverse_ip")
        self.reverse_port = self._get_hw_param("reverse_port")
        self.script_sender_port = self._get_hw_param("script_sender_port")

        assert self.robot_ip
        assert self.reverse_port

        if not self.reverse_ip:
            self.reverse_ip = get_own_ip(self.robot_ip)

    def _get_hw_param(self, param_name):
        if rospy.has_param(f"~{param_name}"):
            rospy.logwarn(
                f"Using directly provided parameter '{param_name}'. If possible, run this node "
                "while a ur_hardware_interface instance is running."
            )
            return rospy.get_param(f"~{param_name}")
        elif rospy.has_param(f"ur_hardware_interface/{param_name}"):
            return rospy.get_param(f"ur_hardware_interface/{param_name}")
        else:
            rospy.logerr(
                f"Could not get parameter '{param_name}'. Either provide it directly or better "
                "have an instance of the driver running."
            )

    def ping(self):
        """Ping a machine once. Returns True on success, False otherwise."""

        return self._check_run(["ping", "-c 1", self.robot_ip])

    def reverse_ping(self):
        """Ping own machine from remote machine"""
        return self._check_run(
            [
                "sshpass",
                "-p",
                self.remote_password,
                "ssh",
                f"{self.remote_user}@{self.robot_ip}",
                f"ping -c 1 {self.reverse_ip}",
            ]
        )

    def reverse_connect(self):
        return self._check_run(
            [
                "sshpass",
                "-p",
                self.remote_password,
                "ssh",
                f"{self.remote_user}@{self.robot_ip}",
                f"nc -zv -w 2 {self.reverse_ip} {self.reverse_port}",
            ]
        )

    def test_request_program(self):
        cmd = [
            "sshpass",
            "-p",
            self.remote_password,
            "ssh",
            f"{self.remote_user}@{self.robot_ip}",
            f"echo 'request_program' | nc -q 1 {self.reverse_ip} {self.script_sender_port}",
        ]

        try:
            result = subprocess.run(cmd, check=True, capture_output=True, text=True)
            return len(result.stdout) > 0
        except subprocess.CalledProcessError as err:
            rospy.logerr(err.stderr.strip())
            return False


    def _check_run(self, cmd):
        """Uses subprocess.check_output() and returns True on success and False if a CalledProcessError
        occurred

        :param cmd: command that should be executed
        """
        try:
            subprocess.run(cmd, check=True, capture_output=True, text=True)
        except subprocess.CalledProcessError as err:
            rospy.logerr(err.stderr.strip())
            return False
        return True

    def _run_check(self, msg, fun, hints):
        rospy.loginfo(msg)
        if fun():
            rospy.loginfo(f"Success: {msg}\n")
        else:
            rospy.logerr(f"FAILED: {msg}")
            hints_str = "This could be due to one of the following reasons:\n"
            for hint in hints:
                hints_str += f"- {hint.strip()}\n"
            rospy.loginfo(hints_str)

    def run_checks(self):
        self._run_check(
            msg=f"Pinging robot's IP address {self.robot_ip}",
            fun=self.ping,
            hints=["The configured robot_ip is probably wrong or the robot is not reachable."],
        )
        self._run_check(
            msg=f"Trying to ping ourself from robot ({self.robot_ip}) using ssh "
            + f"with user {self.remote_user}",
            fun=self.reverse_ping,
            hints=[
                "The robot does not have ssh enabled / installed (e.g. when using a docker image)"
            ],
        )
        self._run_check(
            msg=f"Trying to connect to ourself on port {self.reverse_port}",
            fun=self.reverse_connect,
            hints=[
                "The ur_robot_driver is not running",
                "The robot does not have ssh enabled / installed (e.g. when using a docker image)",
                "This could potentially mean that there is a firewall "
                + f"restricting access to port {self.reverse_port}",
            ],
        )
        self._run_check(
            msg=f"Trying to request program on port {self.script_sender_port}",
            fun=self.test_request_program,
            hints=[
                "The ur_robot_driver is not running",
                "The robot does not have ssh enabled / installed (e.g. when using a docker image)",
                "This could potentially mean that there is a firewall "
                + f"restricting access to port {self.script_sender_port}",
            ],
        )


def main():
    """Main worker function"""
    debugger = ConnectionDebugger()
    debugger.run_checks()


if __name__ == "__main__":
    main()
