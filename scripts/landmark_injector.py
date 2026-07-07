#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import csv
import sys
import time

class LandmarkInjector(Node):
    def __init__(self, csv_file):
        super().__init__('landmark_injector')
        self.publisher_ = self.create_publisher(PointStamped, 'lio_sam/landmark', 10)
        self.csv_file = csv_file
        self.publish_landmarks()

    def publish_landmarks(self):
        # Give LIO-SAM a moment to boot up before firing messages
        time.sleep(2)
        
        try:
            with open(self.csv_file, 'r') as f:
                reader = csv.reader(f)
                header = next(reader)
                
                # NOTE on Projection:
                # LIO-SAM optimizes in a local Cartesian frame (X, Y, Z).
                # If your CSV contains (Lat, Lon, Elevation), you must project it to local (X, Y, Z) first.
                # You can use the `pyproj` library in Python to convert Lat/Lon to UTM, and then 
                # subtract your start location's UTM coordinates to get local (X, Y).
                #
                # For this script, we assume the CSV is already projected to local coordinates:
                # Format: timestamp, x, y, z
                
                count = 0
                for row in reader:
                    if len(row) < 4:
                        continue
                        
                    stamp_sec = float(row[0])
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[3])

                    msg = PointStamped()
                    msg.header.stamp.sec = int(stamp_sec)
                    msg.header.stamp.nanosec = int((stamp_sec - int(stamp_sec)) * 1e9)
                    msg.header.frame_id = "map"
                    
                    msg.point.x = x
                    msg.point.y = y
                    msg.point.z = z

                    self.publisher_.publish(msg)
                    self.get_logger().info(f'Injected landmark at timestamp {stamp_sec:.3f}: x={x:.2f}, y={y:.2f}, z={z:.2f}')
                    count += 1
                    
                    # Small delay so we don't overwhelm the queue instantly
                    time.sleep(0.1)
                    
                self.get_logger().info(f'Successfully injected {count} survey landmarks.')
                
        except Exception as e:
            self.get_logger().error(f'Failed to read CSV: {e}')

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: python3 landmark_injector.py <path_to_landmarks.csv>")
        sys.exit(1)
        
    rclpy.init(args=args)
    injector = LandmarkInjector(sys.argv[1])
    rclpy.spin_once(injector)
    injector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
