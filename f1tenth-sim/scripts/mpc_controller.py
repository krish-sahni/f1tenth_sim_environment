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
from geometry_msgs.msg import PointStamped

class MPCController:
    def __init__(self):
        rospy.init_node('mpc_controller')

        # Vehicle params
        self.wheelbase  = 0.325     # m
        self.max_steer  = np.deg2rad(30)
        self.max_speed  = 0.4       # m/s
        self.max_accel  = 3.0       # m/s²
        self.dt         = 0.1       # s

        # MPC params
        self.N  = 10
        self.Q  = np.diag([50., 50., 20., 5.])
        self.R  = np.diag([50., 5.])
        self.Rd = np.diag([10., 1.])

        #Obstacles
        self.obstacles = [(1.8101,1.6861)]
        self.obstacle_radius = 0.3
        self.safety_margin  = 0.1

        # state + waypoints
        self.current_pose = np.zeros(4)
        self.read_waypoints()   # loads self.waypoints + self.waypoint_tree

        #Obstacle Subscriber
        rospy.Subscriber('/obstacles', PointStamped, self.obstacle_callback)

        # build MPC QP
        self.setup_mpc()


        # ROS interface
        rospy.Subscriber('/car_1/base/odom',
                         Odometry,
                         self.odom_callback)
        self.drive_pub = rospy.Publisher('/car_1/offboard/command',
                                         AckermannDrive,
                                         queue_size=1)

    def obstacle_callback(self, msg):
        """ Keep only the most recent obstacle. """
        self.obstacles = [(msg.point.x, msg.point.y)]


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

        self.hs_param = cp.Parameter((2,3))

        cost = 0
        cons = [ self.x[:,0] == self.x0 ]

        #obstacle avoidance constraints
        for k in range(self.N+1):
            # x_k, y_k
            cons += [
              self.hs_param[0,0]*self.x[0,k]
            + self.hs_param[0,1]*self.x[1,k]
            + self.hs_param[0,2] >= 0,
              self.hs_param[1,0]*self.x[0,k]
            + self.hs_param[1,1]*self.x[1,k]
            + self.hs_param[1,2] >= 0
            ]


        for k in range(self.N):
            cost += cp.quad_form(self.x[:,k]   - self.ref_traj[:,k], self.Q)
            cost += cp.quad_form(self.u[:,k], self.R)
            if k>0:
                cost += cp.quad_form(self.u[:,k]-self.u[:,k-1], self.Rd)

            # linearized dyn
            x_next = ( self.Ad_p @ self.x[:,k]
                     + self.Bd_p @ self.u[:,k]
                     + self.Cd_p )
            cons += [ self.x[:,k+1] == x_next,
                      cp.abs(self.u[0,k])   <= self.max_steer,
                      self.u[1,k]           <= self.max_accel,
                      self.u[1,k]           >= -self.max_accel]
                    #   self.x[3,k]           >= -0.05,
                    #   self.x[3,k]           <= self.max_speed]

        cost += cp.quad_form(self.x[:,self.N] - self.ref_traj[:,self.N], self.Q)
        # print(f"cons: {cons}")
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

    def unwrap_angle(self, angle, prev_angle):
        delta = angle - prev_angle
        while delta > np.pi:
            delta -= 2*np.pi
        while delta < -np.pi:
            delta += 2*np.pi
        return prev_angle + delta

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
                raw_yaw = math.atan2(dy, dx)
                yaw_ref = self.unwrap_angle(raw_yaw, ref[2, k-1])
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

    def compute_halfspaces(self, ox, oy, px, py):
        """Returns two (a,b,c) triples where the first is the left tangent line and the second is a dummy."""
        # Current car position (px, py), yaw from current_pose
        yaw = self.current_pose[2]
        fx = px + 0.2 * math.cos(yaw)
        fy = py + 0.2 * math.sin(yaw)
        R = self.obstacle_radius + self.safety_margin

        def get_tangents(ox, oy, fx, fy, R):
            px_prime = fx - ox
            py_prime = fy - oy
            d_squared = px_prime**2 + py_prime**2
            d = math.sqrt(d_squared)
            if d < R:
                return None, None
            a = px_prime
            b = py_prime
            R_squared = R**2
            sqrt_term = math.sqrt(d_squared - R_squared)
            tx1_prime = (R_squared * a + R * b * sqrt_term) / d_squared
            ty1_prime = (R_squared * b - R * a * sqrt_term) / d_squared
            tx2_prime = (R_squared * a - R * b * sqrt_term) / d_squared
            ty2_prime = (R_squared * b + R * a * sqrt_term) / d_squared
            tx1 = tx1_prime + ox
            ty1 = ty1_prime + oy
            tx2 = tx2_prime + ox
            ty2 = ty2_prime + oy
            return (tx1, ty1), (tx2, ty2)

        tangents = get_tangents(ox, oy, fx, fy, R)
        if tangents[0] is None:
            return (0.0, 0.0, 1e6), (0.0, 0.0, 1e6)
        
        (tx1, ty1), (tx2, ty2) = tangents

        # Determine left tangent using cross product
        dir_vec = (math.cos(yaw), math.sin(yaw))
        vec1 = (tx1 - fx, ty1 - fy)
        cross1 = dir_vec[0] * vec1[1] - dir_vec[1] * vec1[0]
        vec2 = (tx2 - fx, ty2 - fy)
        cross2 = dir_vec[0] * vec2[1] - dir_vec[1] * vec2[0]

        tx, ty = None, None
        if cross1 > 0 and cross2 > 0:
            tx, ty = (tx1, ty1) if cross1 > cross2 else (tx2, ty2)
        elif cross1 > 0:
            tx, ty = tx1, ty1
        elif cross2 > 0:
            tx, ty = tx2, ty2
        else:
            return (0.0, 0.0, 1e6), (0.0, 0.0, 1e6)

        # Line equation through (fx, fy) and (tx, ty)
        a_line = ty - fy
        b_line = -(tx - fx)
        c_line = tx * fy - fx * ty

        # Ensure valid region is outside the obstacle
        value = a_line * ox + b_line * oy + c_line
        if value > 0:
            a_line, b_line, c_line = -a_line, -b_line, -c_line

        return (a_line, b_line, c_line), (0.0, 0.0, 1e6)

    # def compute_halfspaces(self, ox, oy, px, py):
    #     """Returns two (a,b,c) triples."""
    #     # 1) car→obs
    #     dx, dy = ox-px, oy-py
    #     dist = math.hypot(dx,dy)
    #     print(f"dist: {dist}")
    #     if dist < 1e-3:
    #         # no meaningful constraint
    #         return (0,0, self.obstacle_radius+self.safety_margin), (0,0, 1e6)
    #     dx,dy = dx/dist, dy/dist
    #     # 2) perp normal
    #     nx,ny = -dy, dx
    #     # 3) offset so nx*ox + ny*oy + c = margin
    #     margin = self.obstacle_radius + self.safety_margin
    #     c1 = -(nx*ox + ny*oy) + margin
    #     # second halfspace “always true”
    #     return (nx,   ny,   c1), (0.0, 0.0, 1e6)
    
    def odom_callback(self, msg):
        print("[ODOM CALLBACK CALLED]")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _,_,yaw = tf.transformations.euler_from_quaternion(
                     [q.x,q.y,q.z,q.w])
        self.current_pose = np.array([
            x,
            y,
            yaw,
            msg.twist.twist.linear.x
        ])

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

        print(f"obs: {self.obstacles}, x: {x}, y: {y}")
        if self.obstacles:
            ox, oy = self.obstacles[0]
        else:
            # no obstacle → a trivial “always‐true” pair
            self.hs_param.value = np.array([[0,0,1e6],
                                            [0,0,1e6]])
        # if there is one obs:
        if self.obstacles:
            a1,b1,c1, a2,b2,c2 = (*self.compute_halfspaces(ox,oy, x,y)[0],
                                 *self.compute_halfspaces(ox,oy, x,y)[1])
            self.hs_param.value = np.array([[a1,b1,c1],
                                            [a2,b2,c2]])
            print(f"a1,b1,c1: {a1},{b1},{c1}")
            print(f"a2,b2,c2: {a2},{b2},{c2}")
            
        
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
