#!/home/kanat/myenv/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import cvxpy as cp

class UR5MPCController(Node):
    def __init__(self):
        super().__init__('ur5_mpc_controller')
        
        # UR5 has 6 joints
        self.n_joints = 6
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        # Joint state subscription
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Command publisher
        self.joint_cmd_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        
        # Initialize joint positions
        self.q = np.zeros(self.n_joints)
        
        # MPC parameters
        self.horizon = 10  # prediction steps
        self.dt = 0.1      # time step

    def joint_state_callback(self, msg: JointState):
        self.q = np.array(msg.position)

    def mpc_step(self, q_current, q_target):
        """
        Solve a simple linear MPC to move joints towards q_target
        """
        # Optimization variable: joint velocities over horizon
        dq = cp.Variable((self.n_joints, self.horizon))
        
        # Cost function: minimize distance to target
        cost = 0
        q_pred = q_current
        for t in range(self.horizon):
            q_pred = q_pred + dq[:, t]*self.dt
            cost += cp.sum_squares(q_pred - q_target)
            cost += 0.01*cp.sum_squares(dq[:, t])  # small penalty on velocity

        # Constraints: joint limits
        constraints = [dq <= 1.0, dq >= -1.0]  # rad/s limits
        
        # Solve
        problem = cp.Problem(cp.Minimize(cost), constraints)
        problem.solve(solver=cp.OSQP)
        
        if problem.status not in ["infeasible", "unbounded"]:
            # Return first control input
            self.get_logger().info('successfull')
            return dq.value[:, 0]
        else:
            self.get_logger().warn("MPC problem infeasible")
            return np.zeros(self.n_joints)

    def publish_command(self, dq):
        traj_msg = JointTrajectory()
        traj_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = dq.tolist()
        point.velocities = [0.0]*6  # optional
        point.time_from_start.sec = 0
        traj_msg.points.append(point)
        self.joint_cmd_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = UR5MPCController()
    
    target_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    
    rate = node.create_rate(10)
    
    while rclpy.ok():
        rclpy.spin_once(node)
        
        dq = node.mpc_step(node.q, target_position)
        node.publish_command(dq)
        
        rate.sleep()
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
