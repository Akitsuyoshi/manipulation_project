#!/usr/bin/python3
import rclpy
from rclpy.node import Node

class ObjectDetection(Node):
    def __init__(self) -> None:
        super().__init__("object_detection_node")
        self.get_logger().info("Initiating Object Detection Node")
        self.get_logger().info("Initiated Object Detection Node")


def main(args=None) -> None:
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    rclpy.shutdown()


if __name__ == "__main__":
    main()