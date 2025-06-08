#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import cv2
from cv_bridge import CvBridge
import numpy as np

class FireFollower(Node):
    def __init__(self):
        super().__init__('fire_follower')
        
        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.water_pump_publisher = self.create_publisher(Bool, 'water_pump_control', 10)
        
        # Subscribers
        self.image_subscriber = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Fire detection parameters (HSV values for fire/flame colors)
        # Adjust these values based on your fire detection needs
        self.lower_fire1 = np.array([0, 50, 50])     # Lower red range
        self.upper_fire1 = np.array([10, 255, 255])  # Upper red range
        self.lower_fire2 = np.array([170, 50, 50])   # Lower red range (wraps around)
        self.upper_fire2 = np.array([180, 255, 255]) # Upper red range (wraps around)
        self.lower_orange = np.array([10, 100, 100]) # Orange/yellow range
        self.upper_orange = np.array([25, 255, 255]) # Orange/yellow range
        
        # Control parameters
        self.linear_speed = 0.3
        self.angular_speed = 0.5
        self.min_fire_area = 1000  # Minimum area to consider as fire
        self.fire_close_area = 5000  # Area threshold to consider fire is close
        self.image_center_x = 320  # Assuming 640x480 image
        
        self.get_logger().info('Fire Follower Node Started')

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Process the image to detect fire
            fire_detected, fire_center_x, fire_area = self.detect_fire(cv_image)
            
            # Create Twist message for robot movement
            twist = Twist()
            pump_msg = Bool()
            
            if fire_detected:
                self.get_logger().info(f'Fire detected! Area: {fire_area}, Center X: {fire_center_x}')
                
                # Calculate error from center of image
                error = fire_center_x - self.image_center_x
                
                # If fire is close enough, stop and activate water pump
                if fire_area > self.fire_close_area:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    pump_msg.data = True
                    self.get_logger().info('Fire is close! Activating water pump!')
                else:
                    # Move towards the fire
                    twist.linear.x = self.linear_speed
                    
                    # Turn towards the fire (proportional control)
                    twist.angular.z = -error * self.angular_speed / self.image_center_x
                    pump_msg.data = False
                    
            else:
                # No fire detected, stop the robot
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                pump_msg.data = False
                self.get_logger().info('No fire detected')
            
            # Publish commands
            self.cmd_vel_publisher.publish(twist)
            self.water_pump_publisher.publish(pump_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def detect_fire(self, image):
        """
        Detect fire in the image using HSV color space
        Returns: (fire_detected, fire_center_x, fire_area)
        """
        # Convert BGR to HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        # Create masks for fire colors (red and orange/yellow)
        mask1 = cv2.inRange(hsv, self.lower_fire1, self.upper_fire1)
        mask2 = cv2.inRange(hsv, self.lower_fire2, self.upper_fire2)
        mask3 = cv2.inRange(hsv, self.lower_orange, self.upper_orange)
        
        # Combine masks
        fire_mask = cv2.bitwise_or(mask1, mask2)
        fire_mask = cv2.bitwise_or(fire_mask, mask3)
        
        # Apply morphological operations to reduce noise
        kernel = np.ones((5, 5), np.uint8)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_OPEN, kernel)
        fire_mask = cv2.morphologyEx(fire_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fire_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if len(contours) > 0:
            # Find the largest contour (assuming it's the main fire)
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_fire_area:
                # Calculate centroid
                M = cv2.moments(largest_contour)
                if M["m00"] != 0:
                    center_x = int(M["m10"] / M["m00"])
                    return True, center_x, area
        
        return False, 0, 0

def main(args=None):
    rclpy.init(args=args)
    fire_follower = FireFollower()
    
    try:
        rclpy.spin(fire_follower)
    except KeyboardInterrupt:
        pass
    finally:
        fire_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()