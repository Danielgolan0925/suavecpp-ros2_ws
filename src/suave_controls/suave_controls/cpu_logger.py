import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_srvs.srv import Empty
import psutil
import csv
import time
import threading
import os

class CpuMonitorNode(Node):
    def __init__(self):
        super().__init__('cpu_monitor')
        self.declare_parameter('frequency', 5.0)

        self.start_time = time.time()

        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.cpu_usage_data = []  # To store (timestamp, CPU usage) tuples
        self.is_running = True  # To control the CPU monitoring loop

        # Service for exporting data
        self.export_service = self.create_service(Empty, 'exportCPU', self.export_callback)

        # Start monitoring CPU usage in a separate thread
        self.monitor_thread = threading.Thread(target=self.monitor_cpu_usage)
        self.monitor_thread.start()
        self.get_logger().info('CPU Monitor Node has started.')

    def monitor_cpu_usage(self):
        while self.is_running and rclpy.ok():
            # Get CPU usage with a blocking interval for frequency control
            cpu_usage = psutil.cpu_percent(interval=1.0 / self.frequency)
            timestamp = time.time()  # UNIX timestamp
            dt = timestamp - self.start_time
            self.cpu_usage_data.append((timestamp, cpu_usage))
            self.get_logger().info(f'CPU Usage: {cpu_usage}% at {timestamp}')

    def export_callback(self, request, response):
        try:
            csv_filename = time.strftime("CPU_%m_%d_%H_%M_%S.csv")
            csv_file_path = '/home/suave/Data/SuaveMaskingPid/%s' % csv_filename
            if os.path.isfile(csv_file_path):
                self.get_logger().info('File already exist?')
                return response
            with open(csv_file_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Timestamp', 'CPU Usage (%)'])
                writer.writerows(self.cpu_usage_data)
            self.get_logger().info(f'CPU usage data exported to {csv_file_path}')
        except Exception as e:
            self.get_logger().error(f'Failed to export data: {str(e)}')
        return response

    def destroy_node(self):
        self.is_running = False  # Stop the CPU monitoring loop
        self.monitor_thread.join()  # Ensure thread finishes cleanly
        self.export_callback(None, None)  # Export data before shutting down
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CpuMonitorNode()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally: 
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
