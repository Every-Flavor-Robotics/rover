from serial import Serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import time
import board
import adafruit_lsm6ds
import adafruit_lsm6ds.lsm6dsox
import adafruit_lis3mdl
import numpy as np


class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        self.i2c = board.I2C()  # uses board.SCL and board.SDA

        # Define sensor objects
        self.sensor_accel_gyro = adafruit_lsm6ds.lsm6dsox.LSM6DSOX(self.i2c)
        self.sensor_mag = adafruit_lis3mdl.LIS3MDL(self.i2c)

        # Set performance modes
        self.sensor_accel_gyro.accelerometer_range = adafruit_lsm6ds.AccelRange.RANGE_8G
        self.sensor_accel_gyro.gyro_range = adafruit_lsm6ds.GyroRange.RANGE_500_DPS
        self.sensor_accel_gyro.accel_data_rate = adafruit_lsm6ds.Rate.RATE_833_HZ
        self.sensor_accel_gyro.gyro_data_rate = adafruit_lsm6ds.Rate.RATE_833_HZ


        self.sensor_mag.data_rate = adafruit_lis3mdl.Rate.RATE_300_HZ

        # Declare parameters
        # Set default to 80 Hz
        self.declare_parameter("publish_frequency", 240)

        # Create a publisher for the IMU data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5
        )

        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', qos_profile)
        self.publisher_mag = self.create_publisher(MagneticField, 'imu/mag', qos_profile)

        timer_period = 1.0 / self.get_parameter("publish_frequency").value
        self.timer = self.create_timer(timer_period, self.timer_callback)


        self.accel_bias = np.array([0.234138, 0.119748, 9.802475])
        self.gyro_bias = np.array([-0.007139, -0.012046, 0.012216])

        self.mag_hard_iron = np.array([7.794867, -8.142428, -10.143352])
        self.mag_soft_iron = np.array([[0.983671, 0.047410, -0.016287], 
                                       [0.047410, 0.992089, 0.012120], 
                                       [-0.016287, 0.012120, 1.027510]])

    

    def timer_callback(self):
        # Read acceleration, rotation, and magnetometer
        # Time how long it takes to read the data
        accel = self.sensor_accel_gyro.acceleration
        gyro = self.sensor_accel_gyro.gyro
        mag = self.sensor_mag.magnetic

        accel, gyro, mag = self.apply_calibration(np.array(accel), np.array(gyro), np.array(mag))

        # Create a message
        msg_accel_gyro = Imu()
        msg_mag = MagneticField()

        # Populate the message
        msg_accel_gyro.header.stamp = self.get_clock().now().to_msg()
        msg_accel_gyro.header.frame_id = "imu"

        # Acceleration in m/s^2
        msg_accel_gyro.linear_acceleration.x = accel[0]
        msg_accel_gyro.linear_acceleration.y = accel[1]
        msg_accel_gyro.linear_acceleration.z = accel[2]

        # Angular velocity in rad/s
        msg_accel_gyro.angular_velocity.x = gyro[0]
        msg_accel_gyro.angular_velocity.y = gyro[1]
        msg_accel_gyro.angular_velocity.z = gyro[2]

        msg_mag.header = msg_accel_gyro.header

        # Convert from microteslas to teslas
        msg_mag.magnetic_field.x = mag[0] / 1000000
        msg_mag.magnetic_field.y = mag[1] / 1000000
        msg_mag.magnetic_field.z = mag[2] / 1000000

        # Publish the message
        self.publisher_.publish(msg_accel_gyro)
        self.publisher_mag.publish(msg_mag)


    def apply_calibration(self, accel_data: np.ndarray, gyro_data: np.ndarray, mag_data: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:

        gyro_data -= self.gyro_bias 

        # Apply magnetometer calibration
        # First apply hard iron calibration
        mag_data -= self.mag_hard_iron


        # Then apply soft iron calibration
        mag_data = self.mag_soft_iron @ mag_data

        return accel_data, gyro_data, mag_data


def main(args=None):
    rclpy.init(args=args)

    imu_publisher = ImuPublisher()

    rclpy.spin(imu_publisher)

    imu_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()