import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import csv
import os


class WayPointPublisherNode(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.pub_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sub_ = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.timer_period = 1.0
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]
        self.dt = 0.01

        ## Actual state
        self.q = np.zeros(len(self.joint_names))
        self.qd = np.zeros(len(self.joint_names))
        
        # Desired state
        self.q_ref = [1.0, -1.57, 1.57, -1.57, 1.57, 0.0]
        self.qd_ref = np.zeros(len(self.joint_names))
        self.timer_ = self.create_timer(self.timer_period, self.timer_callback)
        self.get_logger().info('WaypointPublisherNode has been started...')
        self.sent = False


    def joint_states_callback(self, msg:JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]
                self.qd[i] = msg.velocity[idx]

    def timer_callback(self):

        if self.sent:
            return
        
        point = JointTrajectoryPoint()
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        point.positions = [1.0, -1.0, -2.0, -2.0, 1.0, 0.0]
        point.velocities = [0.0] * 6
        # point.effort = [0.0] * 6
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.pub_.publish(traj)
        self.get_logger().info('Trajectory is sent...')
        self.sent = True

def main(args=None):
    rclpy.init(args=args)
    node = WayPointPublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()