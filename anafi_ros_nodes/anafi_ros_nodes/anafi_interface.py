#!/usr/bin/env python3

# ros2 run anafi_ros_nodes example --ros-args -r __ns:=/anafi

import rclpy
import sys
import numpy as np
import pymap3d

from rclpy.node import Node
from rclpy.qos import qos_profile_system_default, qos_profile_sensor_data
from std_msgs.msg import Float32
from sensor_msgs.msg import NavSatFix

from rclpy.qos import (
    qos_profile_system_default,
    qos_profile_sensor_data,
)
from geometry_msgs.msg import (
    PoseStamped,
    PointStamped,
    QuaternionStamped,
    Vector3Stamped,
)
from sensor_msgs.msg import NavSatFix
from anafi_uav_msgs.msg import Float32Stamped

GNSS_ORIGIN_DRONE_LAB = (
    63.418215,
    10.401655,
    0,
)  # origin of NED, setting origin to be @ the drone lab at NTNU


class AnafiInterface(Node):
    def __init__(self):
        super().__init__("anafi_interface")

        self.get_logger().info("Anafi ROS Interface is running...")

        self._setup_parameters()
        self._setup_publishers()
        self._setup_subscribers()

        # Variables
        self.msg_pose = PoseStamped()
        self.msg_gps = NavSatFix()
        self.msg_ned = PointStamped()

    def _setup_parameters(self):
        self.roll_cmd_scale = self.declare_parameter("cmd_scale/roll", 1.0).value
        self.pitch_cmd_scale = self.declare_parameter("cmd_scale/pitch", 1.0).value
        self.thrust_cmd_scale = self.declare_parameter("cmd_scale/thrust", 1.0).value
        self.is_qualisys_available = self.declare_parameter(
            "qualisys/available", True
        ).value
        self.is_simulator = self.declare_parameter("simulator/enabled", False).value

        self.get_logger().info(
            "Using scales for roll: "
            + str(self.roll_cmd_scale)
            + ", pitch: "
            + str(self.pitch_cmd_scale)
            + ", yaw: "
            + str(self.thrust_cmd_scale)
        )

        if self.is_qualisys_available:
            self.get_logger().info("Flying at Drone Lab, using Qualisys as origin")
        elif self.is_simulator:
            self.get_logger().info("Flying in simulator")
        else:
            self.get_logger().info("Flying outside, using GNSS as origin")

    def _setup_subscribers(self):
        if self.is_qualisys_available:
            self.create_subscription(
                PoseStamped,
                "qualisys/Anafi/pose_downsampled",
                self.qualisys_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                QuaternionStamped,
                "drone/attitude",
                self.drone_attitude_callback,
                qos_profile_sensor_data,
            )

        else:
            # self.create_subscription(
            #     NavSatFix,
            #     "drone/gps/location",
            #     self.gps_callback,
            #     qos_profile_sensor_data,
            # )
            self.create_subscription(
                NavSatFix,
                "drone/location",  # GPS location from camera metadata
                self.gps_callback,
                qos_profile_sensor_data,
            )
            self.create_subscription(
                PointStamped,
                "drone/position",  # NED position from camera metadata
                self.ned_callback,
                qos_profile_system_default,
            )
            self.create_subscription(
                PoseStamped,
                "drone/pose",  # Pose from camera metadata
                self.pose_callback,
                qos_profile_system_default,
            )
            self.create_subscription(
                PointStamped,
                "home/location",
                self.home_location_callback,
                qos_profile_system_default,
            )
        self.create_subscription(
            Float32,
            "drone/altitude",
            self.alitude_callback,
            qos_profile_sensor_data,
        )

    def _setup_publishers(self):
        """
        Sets up publishers for GNSS and NED location data.

        Creates publishers for GNSS location data and NED location data relative to the GNSS origin.
        """
        self.pub_gnss_location = self.create_publisher(
            NavSatFix, "gnss_location", qos_profile_sensor_data
        )
        self.pub_ned_location = self.create_publisher(
            PointStamped, "ned_pos_from_gnss", qos_profile_system_default
        )
        self.pub_pose_drone = self.create_publisher(
            PoseStamped, "pose", qos_profile_system_default
        )
        self.pub_height_drone = self.create_publisher(
            Float32Stamped, "height", qos_profile_system_default
        )
        self.pub_ned_origin = self.create_publisher(
            Vector3Stamped, "ned_frame_gnss_origin", qos_profile_system_default
        )

    # Callback functions used when anafi is flying outside
    def gps_callback(self, msg):
        """
        Callback function for GPS messages.

        Args:
            msg (sensor_msgs.msg.NavSatFix): The GPS message to be processed.

        Returns:
            None
        """
        self.pub_gnss_location.publish(msg)

    def home_location_callback(self, msg):
        """
        May need to change this to service
        """
        self.pub_ned_origin.publish(msg)

    def ned_callback(self, msg):
        """
        Callback function for NED location messages.

        This function is called whenever a new NED location message is received. It publishes the message to the
        `pub_ned_location` topic.

        Args:
            msg (anafi_ros.msg.Ned): The NED location message.

        Returns:
            None
        """
        self.pub_ned_location.publish(msg)

    def pose_callback(self, msg):
        """
        Callback function for pose messages.

        This function is called whenever a new pose message is received. It publishes the message to the
        `pub_pose_drone` topic.

        Args:
            msg (geometry_msgs.msg.PoseStamped): The pose message.

        Returns:
            None
        """
        self.pub_pose_drone.publish(msg)

    # Callback functions used when anafi is located at lab
    def qualisys_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        noise = np.random.normal(0, 0.05, 3)  # Zero mean, 0.05 std. dev gaussian noise

        x += noise[0]
        y += noise[1]
        z += noise[2]

        ell_wgs84 = pymap3d.Ellipsoid("wgs84")
        lat0, lon0, h0 = GNSS_ORIGIN_DRONE_LAB

        self.msg_ned_origin = Vector3Stamped()
        self.msg_ned_origin.vector.x = lat0
        self.msg_ned_origin.vector.y = lon0
        self.msg_ned_origin.vector.z = h0

        # Convert from LAB NED to LAT/LON/ALT
        lat1, lon1, h1 = pymap3d.ned2geodetic(
            x, y, z, lat0, lon0, h0, ell=ell_wgs84, deg=True
        )

        self.msg_gps.header.frame_id = "world"
        self.msg_gps.latitude = lat1
        self.msg_gps.longitude = lon1
        self.msg_gps.altitude = h1
        self.msg_ned = msg

    def drone_attitude_callback(self, msg):
        self.msg_gps.header.stamp = self.get_clock().now().to_msg()
        self.pub_gnss_location.publish(self.msg_gps)

        self.msg_ned.header.stamp = self.get_clock().now().to_msg()
        self.pub_ned_location.publish(self.msg_ned)

        self.msg_ned_origin.header.stamp = self.get_clock().now().to_msg()
        self.pub_ned_origin.publish(self.msg_ned_origin)

        msg_pose = PoseStamped()
        msg_pose.header.stamp = self.get_clock().now().to_msg()
        msg_pose.header.frame_id = "world"
        msg_pose.pose.position.x = self.msg_gps.latitude
        msg_pose.pose.position.y = self.msg_gps.longitude
        msg_pose.pose.position.z = self.msg_gps.altitude
        msg_pose.pose.orientation = msg.quaternion

        self.pub_pose_drone.publish(msg_pose)

    def alitude_callback(self, msg):
        """
        Altitude from drone's camera metadata: will be ground distance given in meters
        """
        msg_height = Float32Stamped()
        msg_height.header.stamp = self.get_clock().now().to_msg()
        msg_height.header.frame_id = "world"
        msg_height.data = msg.data
        self.pub_height_drone.publish(msg_height)


def main(args=None):
    rclpy.init(args=sys.argv)

    anafi = AnafiInterface()

    rclpy.spin(anafi)

    anafi.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
