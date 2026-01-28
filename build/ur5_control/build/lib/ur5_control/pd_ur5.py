import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import csv
import os


class PDControlNode(Node):

    def __init__(self):
        super().__init__('pd_control')
        self.pub_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.sub_ = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.command_sub = self.create_subscription(JointTrajectory, '/moveit_trajectory', self.trajectory_callback, 10)
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
        self.kp = np.array([5, 5, 5, 5, 5, 5])
        self.kd = np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1])

        ## Actual state
        self.q = np.zeros(len(self.joint_names))
        self.qd = np.zeros(len(self.joint_names))
        
        # Desired state
        self.q_ref = [1.0, -1.0, -2.0, -2.0, 1.0, 0.0]
        # self.q_ref = np.zeros(len(self.joint_names))
        self.qd_ref = np.zeros(len(self.joint_names))
        self.timer_ = self.create_timer(self.timer_period, self.pd_control)
        self.get_logger().info('PDControlNode has been started...')
        self.sent = False

        # # MoveIt usage
        # self.moveit = MoveItPy(node_name='moveit_py')
        # self.planning_component = self.moveit.get_planning_component('arm')
        # self.planning_component.set_goal_state(self.q_ref)

        # Logging data
        self.start_time = self.get_clock().now()
        log_dir = os.path.expanduser('~/ur5_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(
            os.path.join(log_dir, 'pd_joint_log.csv'),
            'w',
            newline=''
        )
        self.csv_writer = csv.writer(self.log_file)
        header = ['time']
        for j in range(6):
            header += [
                f'q{j}', f'qd{j}',
                f'q_ref{j}', f'qd_ref{j}'
            ]
        self.csv_writer.writerow(header)


    def joint_states_callback(self, msg:JointState):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.q[i] = msg.position[idx]
                self.qd[i] = msg.velocity[idx]

    def trajectory_callback(self, ref_trajectory:JointTrajectory):
        self.get_logger().info(f"Received trajectory with {len(ref_trajectory.points)} points.")
        
        self.joint_names = ref_trajectory.joint_names
        
        for i, point in enumerate(ref_trajectory.points):
            self.q_ref = list(point.positions)
            self.qd_ref = list(point.velocities) if point.velocities else None

            self.get_logger().info(f"Point {i}:")
            self.get_logger().info(f" q_ref = {dict(zip(self.joint_names, self.q_ref))}")

            if self.qd_ref is not None:
                self.get_logger().info(f" qd_ref = {dict(zip(self.joint_names, self.qd_ref))}")
            else:
                self.get_logger().info(f" qd_ref = not provided...")

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
    
    def pd_control(self):
        error = self.q_ref - self.q
        d_error = self.qd_ref - self.qd
        du = self.kp * error + self.kd * d_error
        q_cmd = self.q + du * self.dt
        point = JointTrajectoryPoint()
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names
        point.positions = q_cmd.tolist()
        point.velocities = [0] * 6
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.pub_.publish(traj)

        # Logging data
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9

        row = [t]
        for i in range(6):
            row += [
                self.q[i],
                self.qd[i],
                self.q_ref[i],     # or q_cmd[i] if you prefer
                self.qd_ref[i]
            ]

        self.csv_writer.writerow(row)

    def destroy_node(self):
        self.log_file.close()
        # super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PDControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()