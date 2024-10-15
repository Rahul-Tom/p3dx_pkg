#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
import numpy as np 
from sklearn.cluster import DBSCAN

class DynamicObjectFollower(Node):
    def __init__(self):
        super().__init__('dynamic_object_follower')
        self.lidar_subscriber = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.last_tracked_object = None  # Initialize the tracked object
        
    def lidar_callback(self, msg):
        # Process the LiDAR scan data to detect dynamic objects
        detected_objects = self.detect_moving_objects(msg)
        self._logger("The dynamic object method has been called")
        if detected_objects:
            # Get the position of the moving object
            object_position = self.track_object(detected_objects)
            if object_position is not None:
                # Publish a goal pose to Nav2 to follow the object
                self.publish_goal(object_position)
    
    def detect_moving_objects(self, scan_data):
        # Step 1: Preprocess LiDAR scan data to (x, y) coordinates
        angle_min = scan_data.angle_min
        angle_max = scan_data.angle_max
        angle_increment = scan_data.angle_increment
        ranges = scan_data.ranges

        # Convert scan ranges to (x, y) points in the LiDAR's coordinate frame
        points = []
        for i, r in enumerate(ranges):
            angle = angle_min + i * angle_increment
            if r < scan_data.range_max and r > scan_data.range_min:  # Filter out invalid readings
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                points.append([x, y])
        
        points = np.array(points)

        # Step 2: Cluster the points (DBSCAN for example)
        if len(points) == 0:
            return []

        clustering = DBSCAN(eps=0.5, min_samples=5).fit(points)
        labels = clustering.labels_

        # Step 3: Extract the centroid of each cluster
        clusters = {}
        for i, label in enumerate(labels):
            if label == -1:  # Ignore noise points
                continue
            if label not in clusters:
                clusters[label] = []
            clusters[label].append(points[i])

        # Compute the centroids of the detected clusters (moving objects)
        detected_objects = []
        for cluster in clusters.values():
            cluster = np.array(cluster)
            centroid = np.mean(cluster, axis=0)
            detected_objects.append(centroid)

        return detected_objects

    def track_object(self, objects):
        if not objects:
            return None  # No objects detected

        # Step 1: Select the closest object (or use some other heuristic)
        if self.last_tracked_object is None:
            # If no object has been tracked, initialize by choosing the closest object
            self.last_tracked_object = min(objects, key=lambda obj: np.linalg.norm(obj))

        # Step 2: Track the closest object to the previously tracked object
        tracked_object = min(objects, key=lambda obj: np.linalg.norm(obj - self.last_tracked_object))

        # Step 3: Update the tracked object's position
        self.last_tracked_object = tracked_object

        return tracked_object
    
    def publish_goal(self, object_position):
        goal = PoseStamped()
        goal.header.frame_id = 'base_link'
        goal.pose.position.x = object_position[0]  # Correct access to the position (numpy array)
        goal.pose.position.y = object_position[1]  # Correct access to the position
        # Set orientation and other goal parameters as needed
        self.goal_publisher.publish(goal)

def main(args=None):
    rclpy.init(args=args)
    dynamic_object_follower = DynamicObjectFollower()
    rclpy.spin(dynamic_object_follower)
    dynamic_object_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
