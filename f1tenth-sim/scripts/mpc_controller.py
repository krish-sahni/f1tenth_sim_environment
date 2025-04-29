#!/usr/bin/env python3
import os
import csv
import rospy
import numpy as np
import cvxpy as cp
import math
import tf
from scipy.spatial import KDTree
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDrive

class MPCController:
    def __init__(self):
        rospy.init_node('mpc_controller')

        # Vehicle params
        self.wheelbase  = 0.325     # m
        self.max_steer  = np.deg2rad(30)
        self.max_speed  = 0.5       # m/s
        self.max_accel  = 3.0       # m/s²
        self.dt         = 0.1       # s

        # MPC params
        self.N  = 30
        self.Q  = np.diag([10., 10., 5., 1.])
        self.R  = np.diag([100., 10.])
        self.Rd = np.diag([10., 1.])

        

        # state + waypoints
        self.current_pose = np.zeros(4)
        self.read_waypoints()   # loads self.waypoints + self.waypoint_tree

        # build MPC QP
        self.setup_mpc()


        # ROS interface
        rospy.Subscriber('/car_1/base/odom',
                         Odometry,
                         self.odom_callback)
        self.drive_pub = rospy.Publisher('/car_1/offboard/command',
                                         AckermannDrive,
                                         queue_size=1)

    def read_waypoints(self):
        fn = os.path.join(os.path.dirname(__file__),
                        '../waypoints/xyhead_demo_pp.csv')
        pts = []
        with open(fn,'r') as f:
            for row in csv.reader(f):
                x,y = row
                pts.append((float(x), float(y)))

        pts = np.array(pts)
        pts = pts[::5]  # <--- Keep every 5th waypoint only (adjust if needed)

        self.waypoints = pts
        self.waypoint_tree = KDTree(self.waypoints)
        rospy.loginfo(f"Loaded {len(pts)} waypoints after downsampling")

    # def read_waypoints(self):
    #     fn = os.path.join(os.path.dirname(__file__),
    #                       '../waypoints/xyhead_demo_pp.csv')
    #     pts = []
    #     with open(fn,'r') as f:
    #         for row in csv.reader(f):
    #             x,y,_ = row
    #             pts.append((float(x), float(y)))
    #     self.waypoints     = np.array(pts)
    #     self.waypoint_tree = KDTree(self.waypoints)
    #     rospy.loginfo(f"Loaded {len(pts)} waypoints")

    def setup_mpc(self):
        # decision vars
        self.x = cp.Variable((4, self.N+1))
        self.u = cp.Variable((2, self.N))

        # problem parameters
        self.x0       = cp.Parameter(4)
        self.ref_traj = cp.Parameter((4, self.N+1))
        self.Ad_p     = cp.Parameter((4,4))
        self.Bd_p     = cp.Parameter((4,2))
        self.Cd_p     = cp.Parameter(4)

        cost = 0
        cons = [ self.x[:,0] == self.x0 ]
        for k in range(self.N):
            cost += cp.quad_form(self.x[:,k]   - self.ref_traj[:,k], self.Q)
            cost += cp.quad_form(self.u[:,k], self.R)
            if k>0:
                cost += cp.quad_form(self.u[:,k]-self.u[:,k-1], self.Rd)

            # linearized dyn
            x_next = ( self.Ad_p @ self.x[:,k]
                     + self.Bd_p @ self.u[:,k]
                     + self.Cd_p )
            cons += [ self.x[:,k+1] == x_next]
                    #   cp.abs(self.u[0,k])   <= self.max_steer,
                    #   self.u[1,k]           <= self.max_accel,
                    #   self.u[1,k]           >= -self.max_accel,
                    #   self.x[3,k]           >= -0.05,
                    #   self.x[3,k]           <= self.max_speed]

        cost += cp.quad_form(self.x[:,self.N] - self.ref_traj[:,self.N], self.Q)
        print(f"cons: {cons}")
        self.prob = cp.Problem(cp.Minimize(cost), cons)



    # def get_reference_trajectory(self):
    # # Create a temp reference trajectory that makes the car turn right

    #     ref = np.zeros((4, self.N+1))
    #     x0, y0, yaw0, v0 = self.current_pose  # Starting point

    #     radius = 1.0  # meters, radius of the circle
    #     delta_yaw_per_step = np.deg2rad(50)  # 5 degrees per step

    #     for k in range(self.N+1):
    #         yaw_ref = yaw0 + k * delta_yaw_per_step  # yaw increasing each step (right turn)

    #         # Position along the circular arc
    #         x_ref = x0 - radius * np.sin(yaw_ref) + radius * np.sin(yaw0)
    #         y_ref = y0 + radius * np.cos(yaw_ref) - radius * np.cos(yaw0)

    #         ref[:,k] = [x_ref, y_ref, yaw_ref, self.max_speed]

    #     return ref


    def get_reference_trajectory(self):
        _, idx = self.waypoint_tree.query(self.current_pose[:2])
        print(f"closest waypoint: {idx}")

        inds = np.clip(np.arange(idx, idx+self.N+1),
                       0, len(self.waypoints)-1)
        print(f"waypoint indices: {inds}")

        pts = self.waypoints[inds]
        print(f"waypoints: {pts}")
        
        ref = np.zeros((4, self.N+1))
        for k,(x_ref,y_ref) in enumerate(pts):
            if k>0:
                dx,dy = pts[k]-pts[k-1]
                yaw_ref = math.atan2(dy,dx)
            else:
                yaw_ref = self.current_pose[2]
            ref[:,k] = [x_ref, y_ref, yaw_ref, self.max_speed]
        return ref

    def linearize_dynamics(self, x_ref, u_ref):
        θ, v = x_ref[2], x_ref[3]
        δ     = u_ref[0]
        A = np.array([
          [0,0, -v*math.sin(θ),  math.cos(θ)],
          [0,0,  v*math.cos(θ),  math.sin(θ)],
          [0,0,              0,  math.tan(δ)/self.wheelbase],
          [0,0,              0,  0]
        ])
        B = np.array([
          [0,0],
          [0,0],
          [v/(self.wheelbase*math.cos(δ)**2), 0],
          [0,1]
        ])
        self.Ad = np.eye(4) + A*self.dt
        self.Bd = B*self.dt
        # C = f(x_ref,u_ref)*dt - A x_ref dt - B u_ref dt
        f = np.array([
          v*math.cos(θ),
          v*math.sin(θ),
          v*math.tan(δ)/self.wheelbase,
          0
        ])
        self.Cd = f*self.dt - (A@x_ref + B@u_ref)*self.dt

    def odom_callback(self, msg):
        print("[ODOM CALLBACK CALLED]")
        q = msg.pose.pose.orientation
        _,_,yaw = tf.transformations.euler_from_quaternion(
                     [q.x,q.y,q.z,q.w])
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw,
            msg.twist.twist.linear.x
        ])

        # build ref + linearizelectrical & Computer Eng Bldg W 05/14/2025 7:0
        ref = self.get_reference_trajectory()
        self.x0.value       = self.current_pose
        self.ref_traj.value = ref
        if self.current_pose[3] < 0.05:
            self.current_pose[3] = 0.05

        self.linearize_dynamics(self.current_pose,
                                np.array([0.0,0.0]))
        self.Ad_p.value = self.Ad
        self.Bd_p.value = self.Bd
        self.Cd_p.value = self.Cd

        self.prob.solve(solver=cp.OSQP, warm_start=True, eps_abs=1e-3, eps_rel=1e-3, max_iter=5000, verbose=False)
        print(f"accel: {self.u.value[1,0]}, angle: {self.u.value[0,0]}")
        if self.prob.status not in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            rospy.logwarn("MPC solve failed")
            return

        steer = float(self.u.value[0,0])
        accel = float(self.u.value[1,0])
        speed = np.clip(self.current_pose[3] + accel*self.dt,
                        0, self.max_speed)

        cmd = AckermannDrive()
        cmd.steering_angle = -steer
        cmd.speed          = speed
        self.drive_pub.publish(cmd)

if __name__=='__main__':
    try:
        MPCController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
