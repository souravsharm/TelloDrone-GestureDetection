import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from djitellopy import Tello
import cv2
import os

class DroneFlying(Node):
    def __init__(self):
        super().__init__('tello_drone')
        self.drone = Tello()
        self.drone.connect()
        
        self.sub = self.create_subscription(String, 'gestures', self.send_command, 1)


    def send_command(self, msg):
        command = msg.data
        if command == 'Takeoff':
            self.drone.takeoff()
            time.sleep(2)
        elif command == 'Right':
            self.drone.move_right(50)
        elif command == 'Land':
            self.drone.land()

def main(args=None):
    rclpy.init(args=args)
    drone = DroneFlying()
    rclpy.spin(drone)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
