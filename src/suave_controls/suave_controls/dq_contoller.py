import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import pandas as pd
from pyquaternion import Quaternion
from dual_quaternions import DualQuaternion
import numpy as np

class DualQuaternionSubscriber(Node):

    def __init__(self):
        super().__init__('dual_quaternion_subscriber')
        self.subscription_ned = self.create_subscription(
            PoseStamped,
            '/drone/ned_position',
            self.ned_callback,
            10)
        self.subscription_quat = self.create_subscription(
            PoseStamped,
            '/drone/quaternion',
            self.quat_callback,
            10)
        self.publisher_real = self.create_publisher(Float64MultiArray, '/drone/dq_ang_velocity', 10)
        self.publisher_dual = self.create_publisher(Float64MultiArray, '/drone/dq_lin_velocity', 10)
        self.publisher_controller_output = self.create_publisher(Float64MultiArray, '/drone/controller_output', 10)
        self.ned_position = None
        self.quaternion = None

    def ned_callback(self, msg):
        self.ned_position = msg.pose.position
        self.process_data()

    def quat_callback(self, msg):
        self.quaternion = msg.pose.orientation
        self.process_data()

    def process_data(self):
        if self.ned_position and self.quaternion:
            data = {
                'North': [self.ned_position.x],
                'East': [self.ned_position.y],
                'Down': [self.ned_position.z],
                'w': [self.quaternion.w],
                'x': [self.quaternion.x],
                'y': [self.quaternion.y],
                'z': [self.quaternion.z]
            }
            df = pd.DataFrame(data)
            dual_quaternions_df = encode_dual_quaternions(df)
            for _, row in dual_quaternions_df.iterrows():
                dq = DualQuaternion(Quaternion(row['real_w'], row['real_x'], row['real_y'], row['real_z']),
                                    Quaternion(row['dual_w'], row['dual_x'], row['dual_y'], row['dual_z']))
                # self.get_logger().info(f"Dual Quaternion: {dq}")
                controller_output = self.controller(dq)
                self.publish_data(row, controller_output)

    def controller(self, dq):
        # Controller Variables
        # Insert your Sliding Mode Controller logic here
        controller_output = [1.0, 0.0, 0.0, 0.0]  # Replace with actual controller logic
        return controller_output

    def publish_data(self, row, controller_output):
        real_msg = Float64MultiArray()
        dual_msg = Float64MultiArray()
        controller_output_msg = Float64MultiArray()

        real_msg.data = [row['real_w'], row['real_x'], row['real_y'], row['real_z']]  #this is angular velocity
        dual_msg.data = [row['dual_w'], row['dual_x'], row['dual_y'], row['dual_z']]  #this is linear velocity
        controller_output_msg.data = controller_output

        self.publisher_real.publish(real_msg)
        self.publisher_dual.publish(dual_msg)
        self.publisher_controller_output.publish(controller_output_msg)

def encode_dual_quaternions(df):
    data = {
        'real_w': [],
        'real_x': [],
        'real_y': [],
        'real_z': [],
        'dual_w': [],
        'dual_x': [],
        'dual_y': [],
        'dual_z': []
    }

    for _, row in df.iterrows():
        t_x, t_y, t_z = row['North'], row['East'], row['Down']
        q_w, q_x, q_y, q_z = row['w'], row['x'], row['y'], row['z']
        q_r = Quaternion(q_w, q_x, q_y, q_z)
        q_t = Quaternion(0, t_x, t_y, t_z)

        real_part = q_r
        dual_part = 0.5 * q_t * q_r

        data['real_w'].append(real_part.w)
        data['real_x'].append(real_part.x)
        data['real_y'].append(real_part.y)
        data['real_z'].append(real_part.z)
        data['dual_w'].append(dual_part.w)
        data['dual_x'].append(dual_part.x)
        data['dual_y'].append(dual_part.y)
        data['dual_z'].append(dual_part.z)
        
    return pd.DataFrame(data)

def main(args=None):
    rclpy.init(args=args)
    node = DualQuaternionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

