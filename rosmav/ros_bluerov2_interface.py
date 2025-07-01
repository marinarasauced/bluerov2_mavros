#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from pymavlink import mavutil
from mavros_msgs.msg import OverrideRCIn, ManualControl
from std_msgs.msg import Int16
from sensor_msgs.msg import (
    FluidPressure as Pressure,
    Temperature,
    MagneticField,
    Imu,
)


class ROSBluerov2Interface(Node):
    _x = 0
    _y = 0
    _z = 500
    _r = 0
    _buttons = 0

    def __init__(self):
        super().__init__("ros_bluerov2_interface")
        self.declare_parameter("udp_params", "udpin:0.0.0.0:14550")
        self.udp_params = self.get_parameter("udp_params").value

        self.mavlink = mavutil.mavlink_connection(self.udp_params)
        self.mavlink.wait_heartbeat()
        self.get_logger().info(
            f"Heartbeat from system {self.mavlink.target_system}, component {self.mavlink.target_component}"
        )

        # Send heartbeat every 0.1 seconds
        self.create_timer(0.1, self.send_heartbeat)

        # Create timer to check for incoming messages
        self.create_timer(0.1, self.check_messages)

        # Create arming/disarming service
        self.arming_service = self.create_service(
            SetBool, "arming", self.arming_callback
        )

        # Create a subscriber to listen to the /bluerov2/override_rc topic
        self.override_rc_sub = self.create_subscription(
            OverrideRCIn, "override_rc", self.override_rc_callback, 10
        )

        # Create a publisher for the pressure message
        self.pressure_pub = self.create_publisher(Pressure, "bluerov2/pressure", 10)

        # Create a publisher for the temperature message
        self.temperature_pub = self.create_publisher(
            Temperature, "temperature", 10
        )

        # Create a publisher for the magnetic field message
        self.magnetic_field_pub = self.create_publisher(
            MagneticField, "magnetic_field", 10
        )

        # Create a publisher for the heading
        self.heading_pub = self.create_publisher(Int16, "heading", 10)

        # Create a publisher for the IMU message
        self.imu_pub = self.create_publisher(Imu, "imu", 10)

        # Create a subscriber to listen to the /bluerov2/manual_control topic
        self.manual_control_sub = self.create_subscription(
            ManualControl, "manual_control", self.manual_control_callback, 10
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
            self._set_neutral_all_channels()
            self.mavlink.arducopter_disarm()
        response.success = True
        response.message = f"Arming: {request.data}"
        return response

    def destroy_node(self):
        """
        Disarm the vehicle before shutting down the node
        """
        self._set_neutral_all_channels()
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
            self.get_logger().debug(f"Received message: {msg}")
            if msg.get_type() == "SCALED_PRESSURE2":
                self._handle_pressure(msg)
            elif msg.get_type() == "SCALED_IMU2":
                self._handle_imu(msg)
            elif msg.get_type() == "VFR_HUD":
                self._handle_vfr_hud(msg)

    def override_rc_callback(self, msg: OverrideRCIn):
        """
        Callback for the /bluerov2/override_rc subscriber

        See https://www.ardusub.com/developers/rc-input-and-output.html#rc-input
        """
        if len(msg.channels) < 11:
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
            msg.channels[8],
            msg.channels[9],
            msg.channels[10],
        )

    def _set_neutral_all_channels(self):
        """
        Set all channels to neutral values
        """
        neutral_values = [1500] * 8
        self.mavlink.mav.rc_channels_override_send(
            self.mavlink.target_system, self.mavlink.target_component, *neutral_values
        )

    def _handle_pressure(self, msg):
        """
        Handle the pressure message
        """
        self.get_logger().debug(f"Pressure: {msg.press_abs}, {msg.press_diff}")

        pressure_msg = Pressure()
        pressure_msg.header.stamp = self.get_clock().now().to_msg()
        pressure_msg.fluid_pressure = msg.press_abs * 100.0  # Convert to Pa
        self.pressure_pub.publish(pressure_msg)

        temperature_msg = Temperature()
        temperature_msg.header.stamp = self.get_clock().now().to_msg()
        temperature_msg.temperature = (
            msg.temperature / 100.0
        )  # Convert to degrees Celsius
        self.temperature_pub.publish(temperature_msg)

    def _handle_imu(self, msg):
        """
        Handle the IMU message
        """
        self.get_logger().debug(
            f"IMU: {msg.xacc}, {msg.yacc}, {msg.zacc}, {msg.xgyro}, {msg.ygyro}, {msg.zgyro}, {msg.xmag}, {msg.ymag}, {msg.zmag}"
        )

        magnetic_field_msg = MagneticField()
        magnetic_field_msg.header.stamp = self.get_clock().now().to_msg()
        # magnetic field is in mgauss, convert to Tesla
        magnetic_field_msg.magnetic_field.x = msg.xmag * 1e-7
        magnetic_field_msg.magnetic_field.y = msg.ymag * 1e-7
        magnetic_field_msg.magnetic_field.z = msg.zmag * 1e-7
        self.magnetic_field_pub.publish(magnetic_field_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        # acceleration is in mG, convert to m/s^2
        imu_msg.linear_acceleration.x = msg.xacc * 9.81 * 1e-3
        imu_msg.linear_acceleration.y = msg.yacc * 9.81 * 1e-3
        imu_msg.linear_acceleration.z = msg.zacc * 9.81 * 1e-3
        # gyroscope is in mrad/s, convert to rad/s
        imu_msg.angular_velocity.x = msg.xgyro * 1e-3
        imu_msg.angular_velocity.y = msg.ygyro * 1e-3
        imu_msg.angular_velocity.z = msg.zgyro * 1e-3
        self.imu_pub.publish(imu_msg)

    def _handle_vfr_hud(self, msg):
        """
        Handle the VFR HUD message
        """
        self.get_logger().debug(
            f"VFR HUD: {msg.airspeed}, {msg.groundspeed}, {msg.heading}, {msg.throttle}, {msg.alt}"
        )

        heading_msg = Int16()
        heading_msg.data = msg.heading
        self.heading_pub.publish(heading_msg)

    def manual_control_callback(self, msg: ManualControl):
        """
        Send manual control message

        See https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """
        self._x = int(msg.x * 10) if msg.x else self._x
        self._y = int(msg.y * 10) if msg.y else self._y
        # msg.z is between -100 and 100, but the MAVLink message expects a value between 0 and 1000
        self._z = int((msg.z * 10) / 2 + 500) if msg.z else self._z
        self._r = int(msg.r * 10) if msg.r else self._r
        self._buttons = int(msg.buttons) if msg.buttons else self._buttons

        self.mavlink.mav.manual_control_send(
            self.mavlink.target_system,
            self._x,
            self._y,
            self._z,
            self._r,
            self._buttons,
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
