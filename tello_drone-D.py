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
        self.image_counter = 0  # Counter for captured images

    def send_command(self, msg):
        command = msg.data
        x= 50
        if command == 'Takeoff':
            self.drone.takeoff()
            time.sleep(2)
        elif command == 'Right':
            self.drone.move_right(x)
        elif command == 'Forward':
            self.drone.move_forward(x)
        elif command == 'Backward':
            self.drone.move_back(x)
        elif command == 'Battery':
            battery = self.drone.get_battery()
            self.get_logger().info(str(battery))

        elif command == 'Mission':

            x =100
            y =0
            z=50
            speed= 30
            
            self.drone.enable_mission_pads()
            time.sleep(1)
            mid = self.drone.get_mission_pad_id()
            self.get_logger().info(str(mid))
            time.sleep(1)
            if(mid == 1):
                self.drone.go_xyz_speed_mid(x, y, z, speed, mid)
        elif command == 'Flip':
            self.drone.flip_back()
        elif command == 'Capture':
            frame = self.drone.get_frame_read().frame
            image_path = os.path.join('Image', f'image_{self.image_counter}.jpg')
            cv2.imwrite(image_path, frame)
            self.get_logger().info(f'Saved image: {image_path}')
            self.image_counter += 1
            time.sleep(0.5)
        elif command == 'Land':
            self.drone.land()

def main(args=None):
    rclpy.init(args=args)
    drone = DroneFlying()
    rclpy.spin(drone)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
