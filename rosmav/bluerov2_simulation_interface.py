
import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Float64


class BlueROV2SimulationInterface(Node):
    _x = 0
    _y = 0
    _z = 500
    _r = 0
    _buttons = 0

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
                f"thruster{i}/cmd_thrust",
                10
            )
            for i in range(6)
        ]
    

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
    
        thruster_outputs = [
            self._x - self._r,
            self._x + self._r,
            self._y - self._r,
            self._y + self._r,
            self._z,
            self._z,
        ]

        for i in range(6):
            cmd_thrust = Float64()
            cmd_thrust.data = float(max(min(thruster_outputs[i], 1.0), -1.0))
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
