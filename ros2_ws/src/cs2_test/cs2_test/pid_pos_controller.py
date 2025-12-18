import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from crazyflie_interfaces.msg import Position
import math
import numpy as np
import time

# ===================== Helper Functions =====================
def acc2attitude(a_ref, psi, m, g):
    # OBS! Accelerations are in world frame
    ax = a_ref[0]
    ay = a_ref[1]
    az = a_ref[2]

    az_g = az + g

    # Total thrust magnitude in N
    T_ref = m * math.sqrt(ax**2 + ay**2 + az_g**2)

    # Using current Yaw to determine what roll and pitch we need

    # ax*cos(psi) + ay*sin(psi) part is to undo the yaw rotation, that is to go
    # from world frame to body frame. The atan2 is used the roll and pitch.
    theta_ref = math.atan2( ax*math.cos(psi) + ay*math.sin(psi), az_g )
    phi_ref   = math.atan2( ax*math.sin(psi) - ay*math.cos(psi), az_g )

    return phi_ref, theta_ref, T_ref

def pure_pursuit_yaw(pos, path3, L):
    # pos   : [x y z]
    # path3 : Nx3 (numpy array)
    # L     : lookahead distance

    # Quick exit
    if path3 is None or len(path3) == 0:
        return 0.0

    x = pos[0]
    y = pos[1]
    z = pos[2]

    # Find closest point in 3D
    # assuming path3 is numpy array for speed
    dxAll = path3[:,0] - x
    dyAll = path3[:,1] - y
    dzAll = path3[:,2] - z

    dist3 = dxAll*dxAll + dyAll*dyAll + dzAll*dzAll # avoid sqrt
    idx = np.argmin(dist3)

    # Find lookahead point further than L (3D distance)
    L2 = L*L  # compare squared distance
    lookIdx = idx

    for k in range(idx, len(path3)):
        dx = path3[k,0] - x
        dy = path3[k,1] - y
        dz = path3[k,2] - z

        if dx*dx + dy*dy + dz*dz >= L2:
            lookIdx = k
            break
            
    # clamp (Python handles index out of bounds differently, but min keeps it safe)
    lookIdx = min(lookIdx, len(path3) - 1)

    # Compute yaw to lookahead in 2D
    dx = path3[lookIdx,0] - x
    dy = path3[lookIdx,1] - y

    psi_ref = math.atan2(dy, dx)

    # wrap
    psi_ref = math.atan2(math.sin(psi_ref), math.cos(psi_ref))
    return psi_ref

class DiscretePID:
    def __init__(self, kp, ki, kd, output_limit, dt, filter_n=100.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = output_limit
        self.dt = dt
        self.n = filter_n
        
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

    def update(self, error):
        # Proportional
        p_term = self.kp * error
        
        # Integral with clamping
        self.integral += error * self.dt
        
        # Dynamic integrator clamping
        # Prevent windup if output is already saturated
        i_term = self.ki * self.integral
        if abs(i_term) > self.limit:
            i_term = math.copysign(self.limit, i_term)
            self.integral = i_term / self.ki if self.ki != 0 else 0.0

        # Derivative with Low Pass Filter
        # derivative = (error - prev_error) / dt
        # filtered_d = (N * derivative + prev_filtered) / (1 + N*dt)? 
        # Simpler implementation:
        raw_derivative = (error - self.prev_error) / self.dt
        d_term = self.prev_derivative + (self.n * self.dt * (raw_derivative - self.prev_derivative))
        
        # Apply gains
        output = p_term + i_term + (self.kd * d_term)
        
        # Final Saturation
        output = max(min(output, self.limit), -self.limit)
        
        self.prev_error = error
        self.prev_derivative = d_term
        
        return output


# =============================== Controller ===============================
class PIDPositionController(Node):
    def __init__(self):
        super().__init__('pos_controller_node')
        
        # 1. HARDCODED GLOBAL PARAMS (From your Matlab snippet)
        self.m = 0.033
        self.g = 9.81
        self.rate = 50.0
        self.dt = 1.0 / self.rate
        self.yawControl = False

        # 2. HARDCODED GAINS (Kp, Ki, Kd, Limit)
        # We manually create the dictionary so the PIDs can be initialized cleanly      
        self.pid_x = DiscretePID(kp=13.5, ki=0.15, kd=0.15, output_limit=4.0, dt=self.dt)
        self.pid_y = DiscretePID(kp=13.5, ki=0.15, kd=0.15, output_limit=4.0, dt=self.dt)
        self.pid_z = DiscretePID(kp=8.0,  ki=3.0,  kd=0.15, output_limit=4.0, dt=self.dt)
        self.pid_yaw = DiscretePID(kp=1.0, ki=0.0, kd=0.0,  output_limit=3.0, dt=self.dt)
        
        # Store them in a dict for the loop logic later if needed, 
        # or just use self.pid_x directly in the loop. 
        # (I will map them here so your main loop code doesn't break if it uses dict access)
        self.pids = {
            'x': self.pid_x,
            'y': self.pid_y,
            'z': self.pid_z,
            'yaw': self.pid_yaw
        }

        # 3. CONVERSIONS
        self.newton_to_pwm = 1.0 / (0.06 * self.g / 65536.0)
        self.rad_to_deg = 180.0 / math.pi

        # 4. STATE & ROS
        self.current_pos = np.zeros(3)
        self.current_yaw = 0.0
        self.target_pos = np.zeros(3)
        self.target_yaw = 0.0
        
        self.state = "LOCKED" 
        self.unlock_start_time = 0.0
        self.received_first_pose = False
        self.received_first_cmd = False
        
        self.sub_pose = self.create_subscription(PoseStamped, '/cf231/pose', self.pose_cb, 10)
        self.sub_cmd = self.create_subscription(Position, '/cf231/cmd_position', self.cmd_cb, 10)
        self.pub_control = self.create_publisher(Twist, '/cf231/cmd_vel_legacy', 10)
        
        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info(f"Controller started at {self.rate} Hz (Hardcoded Gains)")


    def pose_cb(self, msg):
        self.current_pos[0] = msg.pose.position.x / 100.0
        self.current_pos[1] = msg.pose.position.y / 100.0
        self.current_pos[2] = msg.pose.position.z / 100.0
        
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.received_first_pose = True

    def cmd_cb(self, msg):
        self.target_pos[0] = msg.x
        self.target_pos[1] = msg.y
        self.target_pos[2] = msg.z
        self.target_yaw = msg.yaw
        self.received_first_cmd = True

    def control_loop(self):
        if self.unlock_start_time == 0.0:
            self.unlock_start_time = time.time()
            self.last_pid_time = time.time()
            return

        now = time.time()
        elapsed = now - self.unlock_start_time

        # Arming Sequence (0 to 1s)
        if elapsed < 1.0:
            self.send_twist(0.0, 0.0, 0.0, 0.0)
            self.last_pid_time = now 
            return

        # Ramp Up (1s to 3s)
        # if elapsed < 3.0:
        #     ramp_duration = 2.0
        #     # progress = (elapsed - 1.0) / ramp_duration
        #     # ramp_thrust = progress * 40000.0
        #     self.send_twist(0.0, 0.0, 0.0, 0.0)
        #     self.last_pid_time = now
        #     return

        # PID Control (After 3s)
        dt = now - self.last_pid_time
        self.last_pid_time = now
        
        if dt < 1e-6: return

        a_ref = np.zeros(3)
        a_ref[0] = self.pids['x'].update(self.target_pos[0] - self.current_pos[0])
        a_ref[1] = self.pids['y'].update(self.target_pos[1] - self.current_pos[1])
        a_ref[2] = self.pids['z'].update(self.target_pos[2] - self.current_pos[2])

        phi_ref, theta_ref, T_ref = acc2attitude(a_ref, self.current_yaw, self.m, self.g)
        
        if self.yawControl:
            yaw_err = self.target_yaw - self.current_yaw
            yaw_err = math.atan2(math.sin(yaw_err), math.cos(yaw_err))
            yaw_rate_cmd = self.pids['yaw'].update(yaw_err)
        else:
            yaw_rate_cmd = 0.0

        thrust_pwm = max(0.0, min(T_ref * self.newton_to_pwm, 60000.0))

        self.send_twist(
            float(phi_ref * self.rad_to_deg),
            float(theta_ref * self.rad_to_deg),
            float(yaw_rate_cmd * self.rad_to_deg),
            float(thrust_pwm)
        )

    def send_twist(self, roll, pitch, yaw_rate, thrust):
        msg = Twist()
        msg.linear.y = float(roll)
        msg.linear.x = float(pitch)
        msg.linear.z = float(thrust)
        msg.angular.z = float(yaw_rate)
        self.pub_control.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PIDPositionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


