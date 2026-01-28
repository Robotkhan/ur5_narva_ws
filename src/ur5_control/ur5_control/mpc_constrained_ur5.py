#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import cvxpy as cp

class UR5MPCController(Node):
    def __init__(self):
        super().__init__('ur5_mpc_controller')

        # --- Joint parameters ---
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        self.n_joints = 6

        # --- MPC parameters ---
        self.N = 40         # Horizon
        self.dt = 0.2       # Time step
        self.Q = np.eye(self.n_joints) * 20
        self.R = np.eye(self.n_joints) * 0.1

        # --- Constraints ---
        self.traj_limits = np.array([2.2]*6)  # trajectory velocity limit
        self.goal_tolerance = np.array([0.2]*6)
        self.stopped_velocity_tol = 0.0001

        # --- Desired goal ---
        self.q_goal = np.array([0.0, 1.57, 1.57, 1.57, 1.57, 0.0])

        # --- State ---
        self.q = np.zeros(self.n_joints)
        self.qd = np.zeros(self.n_joints)
        self.last_time = self.get_clock().now()

        # --- ROS interfaces ---
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10
        )
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10
        )

        self.timer = self.create_timer(self.dt, self.control_loop)

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]
                self.qd[i] = msg.velocity[idx]

    def control_loop(self):
        # --- CVXPY variables ---
        q = cp.Variable((self.n_joints, self.N+1))
        qd = cp.Variable((self.n_joints, self.N+1))
        u = cp.Variable((self.n_joints, self.N))

        # --- Constraints ---
        constraints = []
        constraints += [q[:,0] == self.q]
        constraints += [qd[:,0] == self.qd]

        for k in range(self.N):
            # Dynamics: double integrator
            constraints += [q[:,k+1] == q[:,k] + self.dt * qd[:,k]]
            constraints += [qd[:,k+1] == qd[:,k] + self.dt * u[:,k]]
            # Velocity limits
            constraints += [cp.abs(qd[:,k]) <= self.traj_limits]

        # Goal constraints at final step
        constraints += [cp.abs(q[:,self.N] - self.q_goal) <= self.goal_tolerance]
        constraints += [cp.abs(qd[:,self.N]) <= self.stopped_velocity_tol]

        # --- Cost ---
        cost = 0
        for k in range(self.N):
            cost += cp.quad_form(q[:,k] - self.q_goal, self.Q)
            cost += cp.quad_form(u[:,k], self.R)
        cost += cp.quad_form(q[:,self.N] - self.q_goal, self.Q)

        # --- Solve MPC ---
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP, warm_start=True)

        if problem.status not in ["infeasible", "unbounded"]:
            # --- Extract first step ---
            q_next = q.value[:,1]
            qd_next = qd.value[:,1]

            # --- Publish trajectory ---
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.joint_names
            point = JointTrajectoryPoint()
            point.positions = q_next.tolist()
            # point.velocities = qd_next.tolist()
            point.velocities = [0.0]*6
            point.time_from_start.sec = 2
            traj_msg.points.append(point)
            self.traj_pub.publish(traj_msg)
            self.get_logger().info("Successful")
            # return 

        else:
            self.get_logger().warn("MPC problem infeasible!")
            # return

def main(args=None):
    rclpy.init(args=args)
    node = UR5MPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
