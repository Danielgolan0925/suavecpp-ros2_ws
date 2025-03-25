import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion, Vector3
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, MultiArrayLayout
import pandas as pd
import numpy as np
from pyquaternion import Quaternion as PyQuaternion
from dual_quaternions import DualQuaternion

class DualQuaternionController(Node):
    def __init__(self):
        super().__init__('dual_quaternion_controller')
        self.subscription_ned = self.create_subscription(
            Vector3, '/drone/ned_position', self.ned_callback, 10)
        self.subscription_quat = self.create_subscription(
            Quaternion, '/drone/quaternion', self.quat_callback, 10)
        self.publisher_controller_output = self.create_publisher(Float64MultiArray, '/drone/controller_output', 10)
        self.publisher_real = self.create_publisher(Quaternion, '/drone/real_quaternion', 10)
        self.publisher_dual = self.create_publisher(Quaternion, '/drone/dual_quaternion', 10)
        self.publisher_velocity = self.create_publisher(Vector3, '/drone/velocity', 10)
        self.ned_position = None
        self.quaternion = None
        #self.dq_desired_df = self.load_desired_state("dqData1.csv")
        self.current_index = 0  # Index for desired state tracking

    def ned_callback(self, msg):
        self.ned_position = msg
        self.process_data()

    def quat_callback(self, msg):
        self.quaternion = msg
        self.process_data()

#################### Construction of Current state as a Dual Quaternion ####################
    def process_data(self):
        if self.ned_position and self.quaternion:
            self.quaternion = PyQuaternion(self.quaternion.w, self.quaternion.x, self.quaternion.y, self.quaternion.z).unit
            df = pd.DataFrame({
                'North': [self.ned_position.x],
                'East': [self.ned_position.y],
                'Down': [self.ned_position.z],
                'w': [self.quaternion.w],
                'x': [self.quaternion.x],
                'y': [self.quaternion.y],
                'z': [self.quaternion.z]
            })
            dq_current_df = encode_dual_quaternions(df)
            dq_current = DualQuaternion(
                PyQuaternion(dq_current_df.iloc[0]['real_w'], dq_current_df.iloc[0]['real_x'], 
                             dq_current_df.iloc[0]['real_y'], dq_current_df.iloc[0]['real_z']),
                PyQuaternion(dq_current_df.iloc[0]['dual_w'], dq_current_df.iloc[0]['dual_x'], 
                             dq_current_df.iloc[0]['dual_y'], dq_current_df.iloc[0]['dual_z']))
            
            # Desired state
            x_des = 0.5 * 20
            y_des = 0.5 * 5   
            z_des = 0.5 * -20
            
            dq_desired = DualQuaternion(
                PyQuaternion(1, 0, 0 , 0),
                PyQuaternion(0, x_des, y_des, z_des))
            
            self.publish_data(dq_current_df.iloc[0], dq_current, dq_desired)

#################### Dual Quaternion Controller ####################

    def controller(self, dq_current, dq_desired):
        lambda_val = .5
        dq_error = dq_current * dq_desired.inverse()
        dq_log = log_dual_quaternion(dq_error)
        controller_output = -lambda_val * dq_log
        
        # Calculate norms
        dq_error_norm = np.linalg.norm([dq_error.q_r.x, dq_error.q_r.y, dq_error.q_r.z])
        dq_log_norm = np.linalg.norm([dq_log.q_r.x, dq_log.q_r.y, dq_log.q_r.z])
        
        print(f"Norm of dq_error: {dq_error_norm:.4f}, Norm of dq_log: {dq_log_norm:.4f}")   
        
        return [
            controller_output.q_r.w, controller_output.q_r.x, controller_output.q_r.y, controller_output.q_r.z,
            controller_output.q_d.w, controller_output.q_d.x, controller_output.q_d.y, controller_output.q_d.z
        ]
#################### ROS2 Message Formation  ####################

    def publish_data(self, row, dq_current, dq_desired):
        real_msg = Quaternion()
        dual_msg = Quaternion()
        controller_output_msg = Float64MultiArray()
        velocity_msg = Vector3()
        
        real_msg.w = row['real_w']
        real_msg.x = row['real_x']
        real_msg.y = row['real_y']
        real_msg.z = row['real_z']
        
        dual_msg.w = row['dual_w']
        dual_msg.x = row['dual_x']
        dual_msg.y = row['dual_y']
        dual_msg.z = row['dual_z']
        
        controller_output = self.controller(dq_current, dq_desired)
        controller_output_msg.data = controller_output
        
        velocity_msg.x, velocity_msg.y, velocity_msg.z = controller_output[-3:]
        
        self.publisher_real.publish(real_msg)
        self.publisher_dual.publish(dual_msg)
        self.publisher_controller_output.publish(controller_output_msg)
        self.publisher_velocity.publish(velocity_msg)

        # Log the real quaternion message
        #self.get_logger().info(f'Published Real Quaternion: [w: {real_msg.w}, x: {real_msg.x}, y: {real_msg.y}, z: {real_msg.z}]')

#################### Construction of DQ Desired ####################

def encode_dual_quaternions(df):
    data = {'real_w': [], 'real_x': [], 'real_y': [], 'real_z': [],
            'dual_w': [], 'dual_x': [], 'dual_y': [], 'dual_z': []}
    
    for _, row in df.iterrows():
        q_r = PyQuaternion(row['w'], row['x'], row['y'], row['z']).unit
        q_t = PyQuaternion(0, row['North'], row['East'], row['Down'])
        dq = DualQuaternion(q_r, 0.5 * q_t * q_r)
        
        data['real_w'].append(dq.q_r.w)
        data['real_x'].append(dq.q_r.x)
        data['real_y'].append(dq.q_r.y)
        data['real_z'].append(dq.q_r.z)
        data['dual_w'].append(dq.q_d.w)
        data['dual_x'].append(dq.q_d.x)
        data['dual_y'].append(dq.q_d.y)
        data['dual_z'].append(dq.q_d.z)
    return pd.DataFrame(data)

#################### Dual Quaternion Log Operation Definition ####################
def log_dual_quaternion(dq):
    qr, qd = dq.q_r, dq.q_d
    norm_vr = np.linalg.norm([qr.x, qr.y, qr.z])
    if norm_vr > 0:
        theta = 2 * np.arctan2(norm_vr, qr.w)
        axis = np.array([qr.x, qr.y, qr.z]) / norm_vr
        log_qr = PyQuaternion(0, *(theta / 2 * axis))
    else:
        log_qr = PyQuaternion(0, 0, 0, 0)
    q_inv = qr.inverse 
    t = 2 * (qd * q_inv).vector  
    log_dq = DualQuaternion(log_qr, PyQuaternion(0, *t) * 0.5)
    return log_dq

def main(args=None):
    rclpy.init(args=args)
    node = DualQuaternionController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    
    ########## Notes for later ##########
        # Turn up the lambda value to increase the rate of convergence
        # Could be bad quaternions
        # Smaller Lambda
        # Build in speed controller
        # Build in a way to say you have reached where you want to go