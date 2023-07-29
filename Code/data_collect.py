# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import numpy as np
import cv2
import os
import csv
import time

# Define the node for data collection
class DataCollectionNode(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('data_collection_node')

        # Create subscriptions to image and velocity topics
        self.image_subscriber = self.create_subscription(Image, '/color/preview/image', self.image_callback, 10)
        self.cmd_vel_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Initialize velocity message
        self.current_cmd_vel = Twist()

        # Define the data directory and csv file path
        self.data_dir = 'data'  
        self.csv_file = os.path.join(self.data_dir, 'data.csv')

        # Create the data directory if it doesn't exist
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        # Create a CSV file and write the header
        with open(self.csv_file, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(['timestamp', 'image_path', 'linear_x', 'angular_z'])

    # Define the callback function for velocity messages
    def cmd_vel_callback(self, cmd_vel_msg):
        # Save the current velocity message
        self.current_cmd_vel = cmd_vel_msg

    # Define the callback function for image messages
    def image_callback(self, img_msg):
        # Convert the image message to a numpy array and save the image
        img = np.array(img_msg.data, dtype=np.uint8).reshape(img_msg.height, img_msg.width, -1)
        timestamp = time.time()
        image_path = os.path.join(self.data_dir, f"{timestamp:.6f}.png")
        cv2.imwrite(image_path, img)

        # Open the CSV file and append the current data
        with open(self.csv_file, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow([timestamp, image_path, self.current_cmd_vel.linear.x, self.current_cmd_vel.angular.z])

# Define the main function
def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create and spin the data collection node
    data_collection_node = DataCollectionNode()
    rclpy.spin(data_collection_node)

    # Destroy the node and shutdown rclpy
    data_collection_node.destroy_node()
    rclpy.shutdown()

# Run the main function if the script is run directly
if __name__ == '__main__':
    main()
