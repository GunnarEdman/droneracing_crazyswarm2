import rclpy
from rclpy.node import Node
from crazyflie_interfaces.msg import Position
import time
import math
import numpy as np

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        
        self.pub = self.create_publisher(Position, '/cf231/cmd_position', 10)
        self.timer = self.create_timer(0.02, self.timer_callback) # 50Hz
        
        self.start_time = time.time()
        self.get_logger().info("Trajectory Publisher Started")

        # ==========================================
        # 1. SELECT TRAJECTORY (Uncomment one)
        # ==========================================
        
        # --- OPTION A: MANUAL (5 points) ---
        # pos_refs = np.array([
        #     [0.0, 0.0, 1.0],
        #     [0.0, 1.0, 1.0],
        #     [1.0, 1.0, 1.0],
        #     [1.0, 0.0, 1.0],
        #     [0.0, 0.0, 1.0]
        # ])
        # switch_times = np.array([0, 7, 14, 17, 20])

        # --- OPTION B: CIRCLE (Default) ---
        # r = 0.3 # Radius reduced for safety in lab
        # z_height = 1.0
        # n = 200 # More points = smoother
        # theta = np.linspace(0, 2*np.pi, n)
        
        # # x = 0.5 + r*cos(theta) (Centered at 0.5, 0.5 like matlab?)
        # # Let's center it at 0,0 for safety first!
        # x = r * np.cos(theta)
        # y = r * np.sin(theta)
        # z = np.full_like(x, z_height)
        
        # self.pos_refs = np.column_stack((x, y, z))
        # self.switch_times = np.linspace(0, 20, n) # Run over 20 seconds

        # --- OPTION C: SPIRAL ---
        # r = 0.5
        # z0 = 0.5
        # z1 = 1.5
        # n = 200
        # num_turns = 3
        # theta = np.linspace(0, 2 * num_turns * np.pi, n)
        # x = r * np.cos(theta)
        # y = r * np.sin(theta)
        # z = np.linspace(z0, z1, n)
        
        # self.pos_refs = np.column_stack((x, y, z))
        # self.switch_times = np.linspace(0, 30, n)

        # ------------------------------------------

        # --- OPTION D: Simple takeoff and hover ---
        self.pos_refs = np.array([
                    [0.0, 0.0, 0.0],  # Start at 0
                    [0.0, 0.0, -0.5],  # Go to
                    [0.0, 0.0, -0.5],  # Hold 
                    [0.0, 0.0, 0.0]   # Back to 0
                ])
        
        # 0s: start, 2s: go up, 4s: land, 6s: finish
        self.switch_times = np.array([0.0, 2.0, 4.0, 6.0])



    def timer_callback(self):
        # Calculate elapsed time
        t = time.time() - self.start_time
        
        # LOGIC: Find which point we should be at
        # Simulink: "if t < switch_times(i) ... break"
        
        # Find the index where switch_time is just greater than t
        # np.searchsorted finds the first index where elements are >= value
        idx = np.searchsorted(self.switch_times, t)
        
        # Handle end of trajectory
        if idx >= len(self.switch_times):
            idx = len(self.switch_times) - 1
        
        # Get the reference point
        ref = self.pos_refs[idx]
        
        # Publish
        msg = Position()
        msg.x = float(ref[0])
        msg.y = float(ref[1])
        msg.z = float(ref[2])
        msg.yaw = 0.0 # Default facing forward
        
        self.pub.publish(msg)
        
        # Optional: Print status every second
        if idx % 10 == 0:
            # print(f"Time: {t:.2f} | Sending: {ref}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()