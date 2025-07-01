
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64

import math
import numpy as np


class BlueROV2SimulationInterface(Node):
    _x = 0
    _y = 0
    _z = 0
    _r = 0
    _buttons = 0
    _theta = np.pi/4
    _a = np.sin(_theta)
    _A = np.array([
        [-_a, _a, -_a, _a],
        [-_a, -_a, _a, _a],
        [-1, 1, -1, 1],
    ])

    def __init__(self):
        super().__init__("bluerov2_software_interface")

        # Create subscription for user-defined ManualControl messages
        self.manual_control_sub = self.create_subscription(
            ManualControl,
            "manual_control",
            self.manual_control_callback,
        10)

        # Create publishers for Gazebo ROS 2 bridge motor topics
        self.cmd_thrust_pubs = [
            self.create_publisher(
                Float64,
                f"thruster{i + 1}/cmd_thrust",
                10
            )
            for i in range(6)
        ]
    

    def manual_control_callback(self, msg: ManualControl):
        """
        Send manual control message

        Input for each field can range from 100 to -100

        Forward Thrust per motor: ~ +51.5 N
        Reverse Thrust per motor: ~ -40.2 N

        See https://mavlink.io/en/messages/common.html#MANUAL_CONTROL
        """
        self._x = msg.x if not math.isnan(msg.x) else self._x
        self._y = msg.y if not math.isnan(msg.y) else self._y
        self._z = msg.z if not math.isnan(msg.z) else self._z
        self._r = msg.r if not math.isnan(msg.r) else self._r

        def scale_thrust(value: float) -> float:
            """
            Scale input in [-100, 100] to thrust in N.
            Positive maps to [0, 51.5], negative maps to [0, -40.2]
            """
            value = max(min(value, 100.0), -100.0)
            return (value / 100.0) * (51.5 if value >= 0 else 40.2)
    
        # thruster_outputs = [
        #     self._x - self._r,
        #     self._x + self._r,
        #     self._y - self._r,
        #     self._y + self._r,
        #     -self._z,
        #     -self._z,
        # ]

        _B = np.array([self._x, self._y, self._z])
        _F = np.linalg.pinv(_A) @ _B

        thruster_outputs = [
            _F[0], _F[1], _F[2], _F[3],
            -self._z, -self._z
        ]

        for i in range(6):
            cmd_thrust = Float64()
            cmd_thrust.data = scale_thrust(thruster_outputs[i])
            self.cmd_thrust_pubs[i].publish(cmd_thrust)


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
