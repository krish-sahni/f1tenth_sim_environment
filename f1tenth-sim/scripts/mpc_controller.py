#!/usr/bin/env python3

import rospy
import numpy as np
import cvxpy as cp
import math
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDrive
from scipy.spatial import KDTree

class MPCController:
    def __init__(self):
        # ROS Initialization
        rospy.init_node('mpc_controller')
        
        # Vehicle Parameters (F1TENTH specific)
        self.wheelbase = 0.325  # meters
        self.max_steer = np.deg2rad(24.0)  # radians
        self.max_speed = 3.0  # m/s
        self.max_accel = 3.0  # m/s²
        self.dt = 0.1  # time step
        
        # MPC Parameters
        self.N = 10  # Prediction horizon
        self.Q = np.diag([10.0, 10.0, 5.0, 1.0])  # State weights [x, y, yaw, v]
        self.R = np.diag([100.0, 10.0])  # Control weights [steer, accel]
        self.Rd = np.diag([10.0, 1.0])  # Control rate weights
        
        # ROS Subscribers/Publishers
        self.odom_sub = rospy.Subscriber('/car_1/base/odom', Odometry, self.odom_callback)
        self.path_sub = rospy.Subscriber('/path', Path, self.path_callback)
        self.drive_pub = rospy.Publisher('/car_1/offboard/command', AckermannDrive, queue_size=1)
        
        # State Variables
        self.current_pose = np.zeros(4)  # [x, y, yaw, v]
        self.waypoints = None
        self.waypoint_tree = None
        
        # Initialize MPC Problem
        self.setup_mpc()

    def setup_mpc(self):
        # State and control variables
        self.x = cp.Variable((4, self.N + 1))
        self.u = cp.Variable((2, self.N))
        
        # Parameters (updated each iteration)
        self.x0 = cp.Parameter(4)  # Initial state
        self.ref_traj = cp.Parameter((4, self.N + 1))  # Reference trajectory
        
        # Cost function
        objective = 0
        for k in range(self.N):
            objective += cp.quad_form(self.x[:, k] - self.ref_traj[:, k], self.Q)
            objective += cp.quad_form(self.u[:, k], self.R)
            if k > 0:
                objective += cp.quad_form(self.u[:, k] - self.u[:, k-1], self.Rd)
        objective += cp.quad_form(self.x[:, self.N] - self.ref_traj[:, self.N], self.Q)
        
        # Constraints
        constraints = []
        constraints += [self.x[:, 0] == self.x0]
        
        for k in range(self.N):
            # Kinematic bicycle model constraints
            x_k   = self.x[:, k]
            x_kp1 = self.x[:, k+1]
            steer_k = self.u[0, k]
            accel_k = self.u[1, k]

            x_dot = cp.vstack([
            x_k[3] * cp.cos(x_k[2]),
            x_k[3] * cp.sin(x_k[2]),
            x_k[3] * cp.tan(steer_k) / self.wheelbase,
            accel_k
            ])

            constraints += [x_kp1 == x_k + self.dt * x_dot]
            
            # Input constraints
            constraints += [cp.abs(self.u[0, k]) <= self.max_steer]
            constraints += [self.u[1, k] <= self.max_accel]
            constraints += [self.u[1, k] >= -self.max_accel]
            
            # Speed constraints
            constraints += [self.x[3, k] >= 0]
            constraints += [self.x[3, k] <= self.max_speed]

        # Setup OSQP problem
        self.prob = cp.Problem(cp.Minimize(objective), constraints)

    def odom_callback(self, msg):
        # Extract current state from odometry
        quaternion = msg.pose.pose.orientation
        _, _, yaw = tf.transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.twist.twist.linear.x
        ])
        
        # Run MPC if waypoints are available
        if self.waypoints is not None:
            self.run_mpc()

    def path_callback(self, msg):
        # Store waypoints and build KDTree for efficient lookup
        self.waypoints = np.array([[pose.pose.position.x, pose.pose.position.y] 
                                  for pose in msg.poses])
        self.waypoint_tree = KDTree(self.waypoints)

    def get_reference_trajectory(self):
        # Find closest waypoint
        closest_idx = self.waypoint_tree.query(self.current_pose[:2])[1]
        
        # Extract next N+1 waypoints
        ref_indices = np.clip(np.arange(closest_idx, closest_idx + self.N + 1), 
                            0, len(self.waypoints)-1)
        ref_traj = self.waypoints[ref_indices]
        
        # Convert to MPC reference format [x, y, yaw, v]
        ref_traj_full = np.zeros((4, self.N + 1))
        for i in range(self.N + 1):
            dx = ref_traj[i, 0] - ref_traj[i-1, 0] if i > 0 else 0
            dy = ref_traj[i, 1] - ref_traj[i-1, 1] if i > 0 else 0
            ref_traj_full[:, i] = [
                ref_traj[i, 0],
                ref_traj[i, 1],
                np.arctan2(dy, dx) if i > 0 else self.current_pose[2],
                self.max_speed  # Maintain target speed
            ]
        return ref_traj_full

    def run_mpc(self):
        # Get reference trajectory
        ref_traj = self.get_reference_trajectory()
        
        # Update MPC parameters
        self.x0.value = self.current_pose
        self.ref_traj.value = ref_traj
        
        # Solve MPC problem
        self.prob.solve(solver=cp.OSQP, warm_start=True)
        
        if self.prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            rospy.logerr("MPC solve failed!")
            return
            
        # Extract first control input
        steer_cmd = self.u.value[0, 0]
        accel_cmd = self.u.value[1, 0]
        
        # Convert acceleration to speed command
        speed_cmd = np.clip(self.current_pose[3] + accel_cmd * self.dt, 0, self.max_speed)
        
        # Publish control command
        drive_msg = AckermannDrive()
        drive_msg.steering_angle = steer_cmd
        drive_msg.speed = speed_cmd
        self.drive_pub.publish(drive_msg)

if __name__ == '__main__':
    try:
        controller = MPCController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass