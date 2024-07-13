#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from icm20948 import ICM20948
from sensor_msgs.msg import Imu, MagneticField, Temperature
from geometry_msgs.msg import Quaternion, TransformStamped
import tf_transformations
from tf2_ros import TransformBroadcaster
import math
import numpy as np

class ICM20948CLASS(Node):
    def __init__(self):
        super().__init__("imu")

        self.logger = self.get_logger()

        # Parameters
        self.declare_parameter("i2c_address", 0x69)
        i2c_addr = self.get_parameter("i2c_address").get_parameter_value().integer_value
        self.i2c_addr = i2c_addr

        self.declare_parameter("frame_id", "base_link")
        frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        self.frame_id = frame_id

        self.declare_parameter("child_frame_id", "chassis")
        child_frame_id = self.get_parameter("child_frame_id").get_parameter_value().string_value
        self.child_frame_id = child_frame_id

        self.declare_parameter("pub_rate", 30)
        pub_rate = self.get_parameter("pub_rate").get_parameter_value().integer_value
        self.pub_rate = pub_rate

        self.imu = ICM20948(self.i2c_addr)
        self.logger.info("Connected...")

        # Calibration data
        self.accel_bias = np.zeros(3)
        self.gyro_bias = np.zeros(3)
        self.mag_bias = np.zeros(3)
        self.mag_scale = np.ones(3)

        # Collect calibration data
        self.collect_calibration_data()

        self.imu_pub_ = self.create_publisher(Imu, "/imu/data_raw", 10)
        self.mag_pub_ = self.create_publisher(MagneticField, "/imu/mag", 10)
        self.temp_pub_ = self.create_publisher(Temperature, "/imu/temp", 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.pub_clk_ = self.create_timer(1 / self.pub_rate, self.publish_cback)

    def collect_calibration_data(self):
        self.logger.info("Collecting calibration data...")
        accel_data = []
        gyro_data = []
        mag_data = []

        for _ in range(500):
            mag_x, mag_y, mag_z = self.imu.read_magnetometer_data()
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.imu.read_accelerometer_gyro_data()
            
            accel_data.append([acc_x, acc_y, acc_z])
            gyro_data.append([gyro_x, gyro_y, gyro_z])
            mag_data.append([mag_x, mag_y, mag_z])
            time.sleep(0.01)

        accel_data = np.array(accel_data)
        gyro_data = np.array(gyro_data)
        mag_data = np.array(mag_data)

        # Compute biases
        self.accel_bias = np.mean(accel_data, axis=0)
        self.gyro_bias = np.mean(gyro_data, axis=0)
        self.mag_bias = np.mean(mag_data, axis=0)

        # Magnetometer scale calibration (simple example)
        self.mag_scale = np.max(np.abs(mag_data - self.mag_bias), axis=0)

        self.logger.info(f"Accel Bias: {self.accel_bias}")
        self.logger.info(f"Gyro Bias: {self.gyro_bias}")
        self.logger.info(f"Mag Bias: {self.mag_bias}")
        self.logger.info(f"Mag Scale: {self.mag_scale}")

    def publish_cback(self):
        imu_msg = Imu()
        mag_msg = MagneticField()
        temp_msg = Temperature()

        # Read IMU data
        mag_x, mag_y, mag_z = self.imu.read_magnetometer_data()
        acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = self.imu.read_accelerometer_gyro_data()
        temp = self.imu.read_temperature()

        # Apply calibration
        acc_x -= self.accel_bias[0]
        acc_y -= self.accel_bias[1]
        acc_z -= self.accel_bias[2]
        
        gyro_x -= self.gyro_bias[0]
        gyro_y -= self.gyro_bias[1]
        gyro_z -= self.gyro_bias[2]

        mag_x = (mag_x - self.mag_bias[0]) / self.mag_scale[0]
        mag_y = (mag_y - self.mag_bias[1]) / self.mag_scale[1]
        mag_z = (mag_z - self.mag_bias[2]) / self.mag_scale[2]

        # Fill IMU message
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = self.frame_id
        imu_msg.linear_acceleration.x = acc_x * 9.8
        imu_msg.linear_acceleration.y = acc_y * 9.8
        imu_msg.linear_acceleration.z = acc_z * 9.8
        imu_msg.angular_velocity.x = gyro_x * 0.0174
        imu_msg.angular_velocity.y = gyro_y * 0.0174
        imu_msg.angular_velocity.z = gyro_z * 0.0174

        # Fill MagneticField message
        mag_msg.header.stamp = self.get_clock().now().to_msg()
        mag_msg.header.frame_id = self.frame_id
        mag_msg.magnetic_field.x = mag_x
        mag_msg.magnetic_field.y = mag_y
        mag_msg.magnetic_field.z = mag_z

        # Fill Temperature message
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        temp_msg.header.frame_id = self.frame_id
        temp_msg.temperature = temp

        # Publish messages
        self.imu_pub_.publish(imu_msg)
        self.mag_pub_.publish(mag_msg)
        self.temp_pub_.publish(temp_msg)

def main(args=None):
    rclpy.init(args=args)
    icm20948 = ICM20948CLASS()
    rclpy.spin(icm20948)

    icm20948.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
