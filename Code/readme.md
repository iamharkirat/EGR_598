This folder contains all the required codes and workspaces to run this project.

* data_collect.py


    This code is a Python script that creates a ROS (Robot Operating System) node for data collection from a simulated robot. It subscribes to two topics: one for receiving images and another for receiving velocity commands. The code saves the received images as PNG files and logs the timestamp, image file path, linear velocity, and angular velocity in a CSV file.

    Here's a breakdown of the code:

    1. Import necessary libraries: The script begins by importing the required libraries for working with ROS messages (rclpy and Node), handling image and velocity messages (Image and Twist), and for image processing (numpy and cv2), as well as other standard Python libraries (os, csv, and time).
    2. Define the node for data collection: The DataCollectionNode class is defined, which is a subclass of rclpy.node.Node. The constructor (__init__) of this class initializes the node with the name "data_collection_node". It creates subscriptions to the image and velocity topics using the create_subscription method provided by rclpy.node.Node.
    3. Initialize variables and file paths: Inside the class constructor, the script initializes variables for holding the current velocity (self.current_cmd_vel) and defines the data directory and CSV file path (self.data_dir and self.csv_file, respectively).
    4. Create data directory and CSV file: The script checks if the data directory exists and creates it if not. It also creates a new CSV file (data.csv) within the data directory and writes a header row with column names ('timestamp', 'image_path', 'linear_x', 'angular_z').
    5. Define callback functions: Two callback functions are defined to handle incoming messages. The cmd_vel_callback function is called when velocity messages are received and saves the received velocity message in the self.current_cmd_vel variable. The image_callback function is called when image messages are received. It converts the image message to a numpy array, saves the image as a PNG file in the data directory with a timestamp-based filename, and appends the timestamp, image file path, linear velocity, and angular velocity to the CSV file.
    6. Define the main function: The main function initializes the ROS client library (rclpy.init), creates an instance of the DataCollectionNode, spins the node to start processing messages (rclpy.spin), and finally, cleans up by destroying the node and shutting down the ROS client library (destroy_node and rclpy.shutdown).
    7. Run the main function: The script checks if it is being run directly (not imported as a module) using if __name__ == '__main__': and then calls the main() function to start the data collection node.
    
    This code is designed to run as a ROS node on a robot with appropriate topics and message types being published on '/color/preview/image' and '/cmd_vel' for image and velocity data, respectively. When the script is run, it will continuously collect image data and velocity commands from the robot and save them in the specified data directory as images and log them in the CSV file for further analysis or training machine learning models, for example.