#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from pymavlink import mavutil
from mavros_msgs.msg import OverrideRCIn


class ROSBluerov2Interface(Node):
    def __init__(self):
        super().__init__("ros_bluerov2_interface")
        self.declare_parameter("udp_params", "udpin:0.0.0.0:14550")
        self.udp_params = self.get_parameter("udp_params").value

        self.mavlink = mavutil.mavlink_connection(self.udp_params)
        self.mavlink.wait_heartbeat()
        self.get_logger().info(
            "Heartbeat from system %u, component %u"
            % (self.mavlink.target_system, self.mavlink.target_component)
        )

        # Send heartbeat every 0.1 seconds
        self.create_timer(0.1, self.send_heartbeat)

        # Create timer to check for incoming messages
        self.create_timer(0.1, self.check_messages)

        # Create arming/disarming service
        self.arming_service = self.create_service(
            SetBool, "bluerov2/arming", self.arming_callback
        )

        # Create a subscriber to listen to the /bluerov2/override_rc topic
        self.override_rc_sub = self.create_subscription(
            OverrideRCIn, "bluerov2/override_rc", self.override_rc_callback, 10
        )

    def send_heartbeat(self):
        self.mavlink.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )

    def arming_callback(self, request, response):
        """
        Callback for the arming service.
        """
        if request.data:
            self.mavlink.arducopter_arm()
        else:
            self.mavlink.arducopter_disarm()
        response.success = True
        response.message = "Arming: %s" % request.data
        return response

    def destroy_node(self):
        """
        Disarm the vehicle before shutting down the node
        """
        self.mavlink.arducopter_disarm()
        self.mavlink.close()
        super().destroy_node()

    def check_messages(self):
        """
        Check for incoming messages from the vehicle
        """
        while True:
            msg = self.mavlink.recv_msg()
            if msg is None:
                break
            self.get_logger().debug("Received message: %s" % msg)

    def override_rc_callback(self, msg):
        """
        Callback for the /bluerov2/override_rc subscriber
        """
        if len(msg.channels) < 8:
            self.get_logger().warn("Received message with less than 8 channels")
            return
        self.mavlink.mav.rc_channels_override_send(
            self.mavlink.target_system,
            self.mavlink.target_component,
            msg.channels[0],
            msg.channels[1],
            msg.channels[2],
            msg.channels[3],
            msg.channels[4],
            msg.channels[5],
            msg.channels[6],
            msg.channels[7],
        )


def main(args=None):
    rclpy.init(args=args)
    node = ROSBluerov2Interface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
