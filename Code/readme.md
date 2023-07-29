<<<<<<< Updated upstream
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
=======
Lane Following Autonomous Vehicle Project

This repository contains the code and workspaces needed to run the lane-following autonomous vehicle project.

One of the primary codes in this repository is the vel_subscriber which defines a ROS2 node, LaneFollower. This node uses a trained machine learning model to predict and publish linear and angular velocity commands for a robot, based on incoming images of a lane. This component plays a key role in our lane following system, presumably for an autonomous vehicle.

Here's a detailed explanation of the various components:

* vel_subscriber:

    This code provides the primary functionality of lane-following. The breakdown of the code is as follows:
    
    1. Imports: The necessary modules such as ROS2 (rclpy), OpenCV (cv2), TensorFlow, and others are imported at the start of the script.
    2. LaneFollower class definition: A custom ROS2 node class, LaneFollower is defined. It inherits from Node, which is part of the rclpy module.
    3. init function: The constructor of the class initializes several critical components:
        * self.bridge = CvBridge(): An object used to convert ROS Image messages to OpenCV images and vice versa.
        * self.image_subscriber: A subscription to an image topic. The system will call the image_callback function whenever a new message is received on that topic.
        * self.cmd_vel_publisher: A publisher to a Twist type topic, used to send velocity commands.
        * self.model: A pre-trained Keras model is loaded.
        * self.label_encoder: A LabelEncoder object is fitted with angular velocities used during the model training phase.
    4. image_callback function: This function is triggered whenever an image message is received. It converts the ROS Image message into an OpenCV image, preprocesses the image, uses the pre-trained Keras model to predict the angular velocity, decodes the predicted label back to its original angular velocity value, and finally sends the calculated angular velocity command.
    5. preprocess_image function: This function preprocesses images before they are fed into the neural network for prediction. It resizes the image to match the input size of the model, normalizes the pixel values, and expands the image dimensions to include the batch size.
    6. send_velocity_command function: This function generates a Twist message containing a linear and angular velocity, and publishes it on the /cmd_vel topic.
    7. main function: The main function is responsible for initializing the ROS2 communication, creating an instance of the LaneFollower node, spinning the node to prevent it from exiting, and finally cleaning up once the node has been stopped.

    This script is intended to be used in a larger system where images are being published to the /color/preview/image topic by a camera or similar sensor on the robot, and another part of the system is listening to the /cmd_vel topic to receive and execute velocity commands.
>>>>>>> Stashed changes
