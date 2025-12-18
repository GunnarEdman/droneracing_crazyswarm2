
"""
Super simpel test just sending low level commans to the drone
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

def main(args=None):
    rclpy.init(args=args)
    node = Node('motor_test_node')
    
    # CORRECT TOPIC
    cf_name = 'cf231' 
    pub = node.create_publisher(Twist, f'/{cf_name}/cmd_vel_legacy', 10)
    
    node.get_logger().info("STARTING TEST (mimicking Xbox)...")
    
    # SEND ZERO (Critical for Safety Unlock)
    print("Sending ZERO to unlock...", flush=True)
    zero_msg = Twist()
    zero_msg.linear.z = 0.0
    for _ in range(10): # Send for 1 second
        pub.publish(zero_msg)
        time.sleep(0.1)

    # RAMP THRUST
    thrust_val = 13000.0
    msg = Twist()
    msg.linear.z = float(thrust_val)
    msg.linear.x = 0.0 # No pitch needed if enHighLevel is 0
    msg.linear.y = 0.0
    
    start = time.time()
    while time.time() - start < 3.0:
        pub.publish(msg)
        time.sleep(0.01) # 100Hz (Standard rate)
        if (time.time() - start) % 0.5 < 0.02:
            print(f"Sending thrust {int(thrust_val)}...", flush=True)

    # STOP
    stop_msg = Twist()
    stop_msg.linear.z = 0.0
    for _ in range(10):
        pub.publish(stop_msg)
        time.sleep(0.1)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()