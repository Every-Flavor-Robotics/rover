
import rclpy
from rclpy.node import Node
import tf_transformations
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_ros import TransformBroadcaster

import time
import struct
import socket
import numpy as np
import threading







class OdometryPublisher(Node):

    def __init__(self, recv_ip: str, recv_port: int, motorgo_ip: str, motorgo_port: int):
        super().__init__('odom_publisher')

        self.recv_ip = recv_ip
        self.recv_port = recv_port

        self.motorgo_ip = motorgo_ip
        self.motorgo_port = motorgo_port

        # Declare parameters
        # Set default to 80 Hz
        self.declare_parameter("publish_frequency", 80.0)


        self.declare_parameter("wheel_diameter", 0.1)
        self.declare_parameter("wheel_base", 0.3)


        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

       
        self.cmd_vel_subscriber = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, qos_profile)


        self.publisher_ = self.create_publisher(Odometry, 'odom', qos_profile)
        self.tf_broadcaster = TransformBroadcaster(self)
        


        timer_period = 1.0 / self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(timer_period, self.timer_callback)


        # Send wheel commands at 60 Hz
        # Create a thread for sending UDP data
        self.left_wheel_velocity_command = 0.0
        self.right_wheel_velocity_command = 0.0
        self.send_thread = threading.Thread(target=self.send_udp_data, args=(self.motorgo_ip, self.motorgo_port, "Hello!", 60))
        self.send_thread.start()


        self.previous_left_wheel_position = None
        self.previous_right_wheel_position = None

        self.previous_left_wheel_velocity = 0.0
        self.previous_right_wheel_velocity = 0.0

        self.left_wheel_position = None
        self.right_wheel_position = None

        self.left_wheel_velocity = 0.0
        self.right_wheel_velocity = 0.0

        self.x = 0.0
        self.y = 0.0
        self.cur_heading = 0.0

        self.message_updated = False
        self.msg = Odometry()

        self.msg_lock = threading.Lock()

        # Compute position based on wheel encoders
        self.wheel_diameter = 0.12

        # Assume that the wheels are 0.3 m apart
        self.wheel_base = 0.21


    def timer_callback(self):
        # If the message has been updated, publish it
        # Else, do nothing
        if self.message_updated:
            with self.msg_lock:
                self.publisher_.publish(self.msg)
                self.message_updated = False

                # Publish the transform
                transform = TransformStamped()
                transform.header = self.msg.header
                transform.child_frame_id = 'base_link'
                transform.transform.translation.x = self.msg.pose.pose.position.x
                transform.transform.translation.y = self.msg.pose.pose.position.y
                transform.transform.translation.z = self.msg.pose.pose.position.z
                transform.transform.rotation = self.msg.pose.pose.orientation

                self.tf_broadcaster.sendTransform(transform)


    def cmd_vel_callback(self, msg: Twist):
        # Compute wheel velocities from linear and angular velocities

        # Compute the linear component
        # wheel_velocity = linear_velocity / wheel_radius
        linear_component = 2 * msg.linear.x / self.wheel_diameter

        # Compute the angular component
        # wheel_velocity = angular_velocity * (wheel_base / 2) / wheel_radius
        angular_component = msg.angular.z * self.wheel_base / self.wheel_diameter

        # Compute the left and right wheel velocities
        self.left_wheel_velocity_command = linear_component - angular_component
        self.right_wheel_velocity_command = linear_component + angular_component



    def send_udp_data(self, ip: str, port: int, message: str, frequency: float):
        global val
        # Create a UDP socket
        sender_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        print(f"Sending UDP data to {ip}:{port}...")
        try:
            while True:
                # Send a struct containing two floats
                sender_sock.sendto(struct.pack('ff', self.left_wheel_velocity_command, self.right_wheel_velocity_command), (ip, port))

                time.sleep(1 / frequency)
        finally:
            sender_sock.close()


    def compute_odometry(self):
        msg = Odometry()

        # Robot has not initialized, return an empty message
        if(self.previous_left_wheel_position is None or self.previous_right_wheel_position is None):
            return msg

        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.header.stamp = self.get_clock().now().to_msg()


        # Compute the change in wheel positions
        delta_left_wheel_position = self.left_wheel_position - self.previous_left_wheel_position
        delta_right_wheel_position = self.right_wheel_position - self.previous_right_wheel_position

        # print(f"Left Wheel Delta: {delta_left_wheel_position}\t Right Wheel Delta: {delta_right_wheel_position}")

        # Compute the distance traveled by each wheel
        left_wheel_distance = delta_left_wheel_position * self.wheel_diameter / 2
        right_wheel_distance = delta_right_wheel_position * self.wheel_diameter / 2

        # Compute the average distance traveled by the wheels
        delta_distance = (left_wheel_distance + right_wheel_distance) / 2.0


        # Compute the change in heading
        delta_heading = (right_wheel_distance - left_wheel_distance) / self.wheel_base

        # Compute linear and angular velocity
        linear_velocity = (self.left_wheel_velocity + self.right_wheel_velocity) / 2.0
        angular_velocity = (self.right_wheel_velocity - self.left_wheel_velocity) / self.wheel_base / 2


        # Compute the change in x and y
        if(np.isclose(delta_heading, 0.0, atol=1e-5)):
            delta_x = delta_distance * np.cos(self.cur_heading)
            delta_y = delta_distance * np.sin(self.cur_heading)

        else:
            r = delta_distance / delta_heading

            delta_x = r * (-np.sin(self.cur_heading) + np.sin(self.cur_heading + delta_heading))
            delta_y = r * (np.cos(self.cur_heading) - np.cos(self.cur_heading + delta_heading))


        # Update the current x and y
        self.x += delta_x
        self.y += delta_y

        # Update the current heading
        self.cur_heading += delta_heading


        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = 0.0

        # Get the quaternion from the current heading
        quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, self.cur_heading)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        msg.twist.twist.linear.x = linear_velocity
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_velocity

        with self.msg_lock:
            self.msg = msg
            self.message_updated = True

        return msg
    


    

    def start(self):
        # Create a thread to receive UDP data
        recv_thread = threading.Thread(target=self.receive_udp_data)
        recv_thread.start()
    
    def receive_udp_data(self):
        """" Receive UDP data from the specified IP and port.
        :param recv_ip: IP address to receive data from
        :param recv_port: Port to receive data from
        :return: 
        """
        # Create a UDP socket for receiving
        recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Bind the socket to the specified IP and port for receiving
        recv_sock.bind((self.recv_ip, self.recv_port))

        print(f"Listening for UDP data on {self.recv_ip}:{self.recv_port}...")

        try:
            while True:
                # Receive data from the socket
                data, addr = recv_sock.recvfrom(1024)  # buffer size is 1024 bytes
                if len(data) == 16:
                    self.previous_left_wheel_position = self.left_wheel_position
                    self.previous_right_wheel_position = self.right_wheel_position

                    self.previous_left_wheel_velocity = self.left_wheel_velocity
                    self.previous_right_wheel_velocity = self.right_wheel_velocity

                    # Unpack the data
                    self.left_wheel_position, \
                    self.right_wheel_position, \
                    self.left_wheel_velocity, \
                    self.right_wheel_velocity = struct.unpack('ffff', data)

                    # print(f"Received data {addr}: Left Velocity = {self.left_wheel_velocity}, Right Velocity = {self.right_wheel_velocity}, Left Position = {self.left_wheel_position}, Right Position = {self.right_wheel_position}")
                else:
                    print(f"Received unexpected data size from {addr}: {data}")

                self.compute_odometry()

        except KeyboardInterrupt:
            print("UDP receiver terminated.")
        finally:
            recv_sock.close()
        return None

    
    
def main(args=None):
    rclpy.init(args=args)

    odom_node = OdometryPublisher("192.168.0.15", 8008, "192.168.0.242", 8008)
    odom_node.start()


    rclpy.spin(odom_node)

    # Destroy the node explicitly on shutdown
    odom_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()