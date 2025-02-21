import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3, Quaternion
import os
import cv2
import numpy as np
from simple_pid import PID  # type: ignore
import csv
import time
import subprocess

pink_lower = np.array([140, 100, 100])
pink_upper = np.array([170, 255, 255])
purple_lower = np.array([110, 50, 50])
purple_upper = np.array([150, 255, 200])

"""
XGAINS = [1, 0, 0]
YGAINS = [1, 0, 0]
ZGAINS = [7.5, 0, 0]
YAWGAINS = [5.5, 0, 0]

XGAINS = [1, 0, 3]
YGAINS = [1, 0, 0]
ZGAINS = [7.5, 0, 0]
YAWGAINS = [6, 0, 0]

XGAINS = [1, 2, 3]
YGAINS = [1, 0, 0]
ZGAINS = [7.5, 0, 0]
YAWGAINS = [6, 0, 0]
"""

# P, I, D
XGAINS = [1, 1.25, 3]
YGAINS = [1, 0, 0]
ZGAINS = [7.5, 0, 0]
YAWGAINS = [6, 0, 0]


class MaskingPIDPublisher(Node):

    def __init__(self):
        super().__init__('masking_pid_publisher')
        self.publisher_ = self.create_publisher(Quaternion, 'masking_pid_publisher', 10)
        self.bounding_box_data = []

        self.frame_width = 640  # Adjust to your camera frame width
        self.frame_height = 480  # Adjust to your camera frame height
        self.center_x = self.frame_width / 2
        self.center_y = self.frame_height / 2
        self.setpoint_depth = 25  # Example setpoint for depth in pixels

        self.start_time = time.time()

        self.pid_x = PID(XGAINS[0], XGAINS[1], XGAINS[2], setpoint=self.center_x)  # PID for horizontal position
        self.pid_y = PID(YGAINS[0], YGAINS[1], YGAINS[2], setpoint=self.center_y)  # PID for vertical position
        self.pid_depth = PID(ZGAINS[0], ZGAINS[1], ZGAINS[2], setpoint=self.setpoint_depth)  # PID for depth position
        self.pid_yaw = PID(YAWGAINS[0], YAWGAINS[1], YAWGAINS[2], setpoint=0.0)

        self.pid_x.output_limits = (-100, 100)  # Adjust as needed for your application
        self.pid_y.output_limits = (-100, 100)  # Adjust as needed for your application
        self.pid_depth.output_limits = (-100, 100)  # Adjust as needed for your application
        self.pid_yaw.output_limits = (-100, 100)

        self._srv_export = self.create_service(Empty, 'exportXYZ', self.service_export)
        self.serials = {}
        self.FILTER = "ID_SERIAL="

    def match_camID_to_serial(self, target_serial):
        """Find the camera by its serial number and return the corresponding camera ID."""
        self.serials = {}  # Initialize the serials dictionary

        # Loop through potential camera IDs (adjust range if needed)
        for cam_id in range(0, 9):
            serial = self.get_cam_serial(cam_id)  # Fetch the serial for each cam_id
            if len(serial) > 6:  # Check if the serial number is valid
                self.serials[cam_id] = serial  # Store the serial number in the serials dictionary
                # print(f"Serials: {self.serials}") UNCOMMENT FOR SERIAL RECOGNITION

        # Now check if target_serial matches any serial in self.serials
        for cam_id, serial in self.serials.items():
            # print(f"Serial attempt to match with {serial}") UNCOMMENT FOR ERROR
            if serial == target_serial:
                print(f"This is the correct camera ID: {cam_id}")
                return cam_id  # Return the matching camera ID

        print("MATCHID Just Returned None which means the serials.items() does not contain target_serial")
        return None  # If no match is found
    #Gets all serial numbers for every camera device connected
    def get_serial_nums(self):
        self.serials = {}  # Make sure the serials dictionary is properly initialized
        for cam_id in range(0, 9):  # Adjust the range if necessary
            self.serial = self.get_cam_serial(cam_id)  # Fetch the serial for each cam_id
            if len(self.serial) > 6:  # Ensure a valid serial number
                self.serials[cam_id] = self.serial  # Store the serial number in self.serials
        print(f"Serials: {self.serials}")
        return self.serials

    def get_cam_serial(self, cam_id):
        """Fetch the serial number for each camera device using udevadm."""
        p = subprocess.Popen(
            f'udevadm info --name=/dev/video{cam_id} | grep {self.FILTER} | cut -d "=" -f 2',
            stdout=subprocess.PIPE, shell=True)
        output, err = p.communicate()
        p.status = p.wait()
        response = output.decode('utf-8')
        # print(f"Serial: {response}")
        return response.replace('\n', '')

    def run(self, camera_index=0):
        # self.match_camID_to_serial("YJX_USB_Camera_20181208")
        camera_id = self.match_camID_to_serial("YJX_USB_Camera_20181208")
        if camera_id is not None:
            cap = cv2.VideoCapture(camera_id)  # Use the camera ID returned from the function
            if not cap.isOpened():
                print(f"Failed to open camera with ID {camera_id}")
            else:
                print(f"Successfully opened camera with ID {camera_id}")
        else:
            print(f"Camera with serial YJX_USB_Camera_20181208 not found.")


        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # Convert frame to HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Create masks for pink and purple
            pink_mask = cv2.inRange(hsv, pink_lower, pink_upper)
            purple_mask = cv2.inRange(hsv, purple_lower, purple_upper)

            # Combine masks
            combined_mask = cv2.bitwise_or(pink_mask, purple_mask)

            # Find contours for the combined mask
            contours, _ = cv2.findContours(combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                # Find the largest contour by area
                largest_contour = max(contours, key=cv2.contourArea)

                # Get bounding box for the largest contour
                x, y, w, h = cv2.boundingRect(largest_contour)
                center_x_box = x + w // 2
                center_y_box = y + h // 2

                # Calculate depth using bounding box width (example proxy for depth)
                depth = w

                # Draw bounding box and center
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.circle(frame, (center_x_box, center_y_box), 5, (255, 0, 0), -1)
                
                # Separate bounding boxes for pink and purple to calculate yaw error
                pink_contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                purple_contours, _ = cv2.findContours(purple_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                pink_width = 0
                purple_width = 0

                if pink_contours:
                    pink_contour = max(pink_contours, key=cv2.contourArea)
                    px, py, pw, ph = cv2.boundingRect(pink_contour)
                    pink_width = pw
                    cv2.rectangle(frame, (px, py), (px + pw, py + ph), (0, 255, 255), 2)  # Yellow for pink box

                if purple_contours:
                    purple_contour = max(purple_contours, key=cv2.contourArea)
                    prx, pry, prw, prh = cv2.boundingRect(purple_contour)
                    purple_width = prw
                    cv2.rectangle(frame, (prx, pry), (prx + prw, pry + prh), (255, 0, 255), 2)  # Magenta for purple box                

                # Compute the control outputs using PID
                control_x = 0
                if (center_x_box is not None):
                    control_x = self.pid_x(center_x_box)
                
                control_y = 0
                if (center_y_box is not None):
                    control_y = self.pid_y(center_y_box)

                control_depth = 0
                if (depth is not None):
                    control_depth = self.pid_depth(depth)

                control_yaw = 0
                yaw_error = 0
                if (pink_width is not None and purple_width is not None):
                    yaw_error = pink_width - purple_width
                    xratio = 1 - abs(center_x_box - self.center_x) // self.center_x
                    yaw_error *= xratio
                    control_yaw = self.pid_yaw(yaw_error)

                # Output the center x, center y, depth, and control values to the terminal
                try:
                    print(f'Center X: {center_x_box:.2f}\tControl X: {control_x:.2f}')
                    print(f'Center Y: {center_y_box:.2f}\tControl Y: {control_y:.2f}')
                    print(f'Depth: {depth:.2f}\t\tControl Depth: {control_depth:.2f}')
                    print(f'Yaw Error: {yaw_error:.2f}\t\tControl Yaw: {control_yaw:.2f}')
                    
                    currTime = time.time()
                    dt = currTime - self.start_time
                    bounding_box_info = {
                        'time': currTime,
                        'width': w,  
                        'height': h,  
                        'center_x': center_x_box,
                        'center_y': center_y_box,
                        'yaw_error': yaw_error,
                        'pid_x': control_x,
                        'pid_y': control_y,
                        'pid_depth': control_depth,
                        'pid_yaw': control_yaw
                    }
                    self.bounding_box_data.append(bounding_box_info)
                except:
                    print("Something went wrong printing...")

                msg = Quaternion() # hack: using quaternion as a vector4
                msg.x = float(control_x)
                msg.y = float(control_y)
                msg.z = float(control_depth)
                msg.w = float(control_yaw)

                self.publisher_.publish(msg)
                self.get_logger().info('Publishing masking pid publisher!')
                rclpy.spin_once(self, timeout_sec=0)

                #self.get_logger().info(f'Bounding box data collected: {bounding_box_info}')


            # Display the resulting frame and mask
            #cv2.imshow('Live Stream with Bounding Box and Center Point', frame)
            #cv2.imshow('Mask', mask)
             # Display the frame and the combined mask
            #cv2.imshow("Bounding Box Tracking", frame)
            #cv2.imshow("Combined Mask", combined_mask)

            # Press 'q' to quit the livestream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

    def service_export(self, request, response):
        self.get_logger().info('Export service called.')

        print("Exporting data as CSV")

        csv_filename = time.strftime("XYZ_%m_%d_%H_%M_%S.csv")
        csv_file_path = '/home/suave/Data/SuaveMaskingPid/%s' % csv_filename
        file_exists = os.path.isfile(csv_file_path)

        if file_exists:
            self.get_logger().info('File already exist?')
            return response
        
        with open(csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(['time', 'Width', 'Height', 'Center X', 'Center Y', 'Yaw Error', 'PID X', 'PID Y', 'PID Depth', 'PID Yaw'])
            for data in self.bounding_box_data:
                writer.writerow([
                    data['time'],
                    data['width'],
                    data['height'],
                    data['center_x'],
                    data['center_y'],
                    data['yaw_error'],
                    data['pid_x'],
                    data['pid_y'],
                    data['pid_depth'],
                    data['pid_yaw']
                ])  
        self.get_logger().info('Exported bounding box data to CSV file!')
        os.system("cp %s /home/suave/Data/SuaveMaskingPid/latest.csv" % csv_file_path)
        return response

def main(args=None):
    print("[main] suave_controls/masking_pid_publisher.py")

    rclpy.init(args=args)

    maskingPIDPublisher = MaskingPIDPublisher()

    maskingPIDPublisher.run()

    maskingPIDPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
