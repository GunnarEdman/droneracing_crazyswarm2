import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import FullState
import time

rclpy.init()
node = Node("formation_move")
pub = node.create_publisher(FullState, "/cf231/cmd_full_state", 10)

square_points = [
    (0.0, 0.0),
    (1.0, 0.0),
    (1.0, 1.0),
    (0.0, 1.0)
]

z_hover = 0.5
linear_speed = 0.4


try:
    while rclpy.ok():
        for x, y in square_points:
            msg = FullState()
            msg.pose.position.x = x
            msg.pose.position.y = y
            msg.pose.position.z = z_hover
            msg.twist.linear.x = linear_speed
            msg.twist.linear.y = linear_speed
            msg.twist.linear.z = 0.0
            pub.publish(msg)
            node.get_logger().info(f"Sent move: X={x:.1f}, Y={y:.1f}, Z={z_hover:.1f}")
            rclpy.spin_once(node, timeout_sec=1.5)
except KeyboardInterrupt:
    pass

rclpy.shutdown()