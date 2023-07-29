# Import the necessary libraries
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow import keras
from sklearn.preprocessing import LabelEncoder

# Define the LaneFollower node
class LaneFollower(Node):

    def __init__(self):
        super().__init__('lane_follower')
        self.bridge = CvBridge()

        # Create subscriptions to the image topic and create a publisher to the velocity command topic
        self.image_subscriber = self.create_subscription(Image,'/color/preview/image',self.image_callback,10)
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Load the trained keras model
        self.model = keras.models.load_model('/home/ani/final_ws/my_model.h5')

        # Initialize the label encoder with the angular_z values used during training
        self.label_encoder = LabelEncoder()
        self.label_encoder.fit([-0.079766, 0.00000, 0.079766])

    def image_callback(self, msg):
        # Convert the image message to cv2 image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Preprocess the image
        preprocessed_image = self.preprocess_image(cv_image)

        # Use the model to predict the angular_z probability
        angular_z_prob = self.model.predict(preprocessed_image)

        # Convert the predictions to label indices
        angular_z_indices = np.argmax(angular_z_prob, axis=1)

        # Convert the label indices back to original angular_z values
        angular_z = self.label_encoder.inverse_transform(angular_z_indices)

        # Get the first (and only) value in the array
        angular_z = angular_z.item(0)

        # Send velocity command
        self.send_velocity_command(angular_z)

    def preprocess_image(self, image):
        # Resize the image to the size expected by the model
        new_width = 224
        new_height = 224
        preprocessed_image = cv2.resize(image, (new_width, new_height))
        
        # Normalize the image pixel values
        preprocessed_image = preprocessed_image.astype('float32') / 255.0

        # Expand the dimensions of the image to include the batch size
        preprocessed_image = np.expand_dims(preprocessed_image, axis=0)
        return preprocessed_image

    def send_velocity_command(self, angular_z):
        # Initialize a Twist message
        twist = Twist()

        # Set the linear and angular velocities
        twist.linear.x = 0.03  # Adjust the linear velocity according to your preference
        twist.angular.z = angular_z

        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)
        print("Published angular_z:", angular_z)

# Define the main function
def main(args=None):
    rclpy.init(args=args)

    # Initialize and spin the lane_follower node
    lane_follower = LaneFollower()
    rclpy.spin(lane_follower)

    # Destroy the node and shutdown rclpy
    lane_follower.destroy_node()
    rclpy.shutdown()

# Run the main function if the script is run directly
if __name__ == '__main__':
    main()
