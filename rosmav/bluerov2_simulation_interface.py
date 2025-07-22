
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64, Int16
from sensor_msgs.msg import MagneticField

import math
import numpy as np


class BlueROV2SimulationInterface(Node):
    _x = 0
    _y = 0
    _z = 0
    _r = 0
    _buttons = 0

    # Remap control input u=[x,y,z,r] to thrust contributions by motor
    _theta = np.pi/4
    _a = np.sin(_theta)
    _b = 1.0
    _A = np.array([
        [-_a, _a, 0, _b],
        [-_a, -_a, 0, -_b],
        [_a, _a, 0, -_b],
        [_a, -_a, 0, _b],
        [0, 0, 1, 0],
        [0, 0, 1, 0],
    ])

    def __init__(self):
        super().__init__("bluerov2_software_interface")

        self.declare_parameter("thrust_min", 40.2)
        self.declare_parameter("thrust_max", 51.5)

        self.thrust_min = self.get_parameter("thurst_min").value
        self.thrust_max = self.get_parameter("thrust_min").value # ignore thrust_max

        # Create subscription for user-defined ManualControl messages
        self.manual_control_sub = self.create_subscription(
            ManualControl,
            "manual_control",
            self.manual_control_callback,
            10
        )

        # Create publishers for Gazebo ROS 2 bridge motor topics
        self.cmd_thrust_pubs = [
            self.create_publisher(
                Float64,
                f"thruster{i + 1}/cmd_thrust",
                10
            )
            for i in range(6)
        ]

        self.magnetic_field_sub = self.create_subscription(
            MagneticField,
            "magnetic_field",
            self.magnetic_field_callback,
            10
        )
        self.heading_pub = self.create_publisher(Int16, "heading", 10)
    

    def manual_control_callback(self, msg: ManualControl):
        """
        Send manual control message

        Input for each field can range from 100 to -100

        Forward Thrust per motor: ~ +{self.thrust_max} N
        Reverse Thrust per motor: ~ -{self.thrust_min} N

        See https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """
        self._x = msg.x if msg.x else self._x
        self._y = msg.y if msg.y else self._y
        self._z = msg.z if msg.z else self._z
        self._r = msg.r if msg.r else self._r

        def scale_thrust(value: float) -> float:
            """
            Scale input in [-100, 100] to thrust in N.
            Positive maps to [0, 51.5], negative maps to [0, -40.2]
            """
            value = max(min(value, 100.0), -100.0)
            return (value / 100.0) * (self.thrust_max if value >= 0 else self.thrust_min)
    
        _B = np.array([self._x, self._y, self._z, self._r])
        _F = self._A @ _B

        thruster_outputs = [
            _F[0], _F[1], _F[2], _F[3],
            -self._z, -self._z
        ]

        for i in range(6):
            cmd_thrust = Float64()
            cmd_thrust.data = scale_thrust(thruster_outputs[i])
            self.cmd_thrust_pubs[i].publish(cmd_thrust)


    def magnetic_field_callback(self, msg: MagneticField):
        """
        Subscribes to Magnetic Field from sim and republishes heading
        """
        mag_x = msg.magnetic_field.x
        mag_y = msg.magnetic_field.y

        heading_rad = math.atan2(mag_y, mag_x)
        heading_deg = math.degrees(heading_rad)
        if heading_deg < 0:
            heading_deg += 360

        heading_msg = Int16()
        heading_msg.data = int(heading_deg)
        self.heading_pub.publish(heading_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BlueROV2SimulationInterface()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
