#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from math import atan2, degrees

class DroneNavigation(Node):

    def __init__(self,latitude,longitude):
        super().__init__('drone_navigation')

       
        self.bridge = CvBridge()

        
        self.lidar_subscriber = self.create_subscription(LaserScan,'/lidar_topic', self.lidar_callback,10)
        self.gps_subscriber = self.create_subscription(NavSatFix,'/gps/fix', self.gps_callback,10)
        self.color_subscriber = self.create_subscription(Image'/color_camera/image_raw', self.color_callback,10)
        self.communication_subscriber = self.create_subscription(String,'/communication',  self.communication_callback,10)

       
        self.stop_publisher = self.create_publisher(Bool, '/stop_search', 10)
        self.twist_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.color_publisher = self.create_publisher(Bool, '/is_green', 10)

        self.target_detected = False
        self.search_active = True
        self.current_gps = None
        self.waypoints = [latitude, longitude]  
        self.current_waypoint_index = 0

    def lidar_callback(self, msg):
        
        if not self.search_active:
            return
        object_detected = self.detect_obstacle(msg)
        if object_detected :
            self.stop_drone()
            self.change_direction()
        else:
            self.navigate_to_waypoint()

    def gps_callback(self, msg):
        
        self.current_gps = msg #getting current location from gps module via the topic/gps/fix

    def color_callback(self, msg):
        if not self.search_active:
            return
        is_green = self.detect_green_color(msg)
        if is_green:# if the detected color is green then we need to send the message to other drones and should stop searching
            self.target_detected = True
            self.notify_other_drones()
            self.stop_search()

    def communication_callback(self, msg):
        if msg.data == "target_found":
            self.get_logger().info("Target found by another drone. Stopping search.")
            self.stop_search()

    def detect_obstacle(self, msg):

        threshold_distance = 0.5  # in meters
        detected_distance = min(msg.ranges)  
        return detected_distance < threshold_distance # checking whether if any side of the drone has object near it.

    def navigate_to_waypoint(self):
       
        if self.current_gps and self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]
            distance = self.calculate_gps_difference(self.current_gps, waypoint)
            if distance == 0.0:  # If within 1 meter of the waypoint
                self.stop_drone()
                self.resume_search()
            else:
                twist = Twist()
                twist.linear.x = 1.0  # Adjust speed as needed
                twist.angular.z = self.bearing_to_angular_velocity(bearing)
                self.twist_publisher.publish(twist)

    def get_distance_from_lat_lon_in_km(self,lat1, lon1, lat2, lon2):
    
        R = 6371000  # Radius of the Earth in meters
        dLat = deg2rad(lat2 - lat1)
        dLon = deg2rad(lon2 - lon1)
    
        a = (math.sin(dLat / 2) ** 2 + 
            math.cos(deg2rad(lat1)) * math.cos(deg2rad(lat2)) * 
            math.sin(dLon / 2) ** 2)
    
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = R * c  # Distance in meters
    
        return d

    def deg2rad(self,deg):
     
        return deg * (math.pi / 180)



    def stop_drone(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.twist_publisher.publish(twist)
    def change_direction(self):
        twist = Twist()
        twist.linear.x = 0.5 
        twist.angular.z = 2.0
        self.twist_publisher.publish(twist)

    def resume_search(self):
        
        twist = Twist()
        twist.linear.x = 0.5  # Adjust speed as needed
        twist.angular.z = 0.9
        self.twist_publisher.publish(twist)

    def stop_search(self):
        # Stop the search process
        self.search_active = False
        stop_msg = Bool(data=True)
        self.stop_publisher.publish(stop_msg)

    def detect_green_color(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return False

       
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_green = np.array([35, 40, 40])
        upper_green = np.array([85, 255, 255])

  
        mask = cv2.inRange(hsv_image, lower_green, upper_green)

      
        green_pixels = cv2.countNonZero(mask)
        total_pixels = cv_image.size / 3  

        green_percentage = (green_pixels / total_pixels) * 100

        self.get_logger().info(f"Green pixel percentage: {green_percentage:.2f}%")

       
        green_threshold = 40.0  

        return green_percentage > green_threshold

    def notify_other_drones(self):
        # Notify other drones about the target
        comm_msg = String(data="target_found")
        self.communication_subscriber.publish(comm_msg)

def main(args=None):
    rclpy.init(args=args)
    drone_navigation = DroneNavigation()
    rclpy.spin(drone_navigation)
    drone_navigation.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
