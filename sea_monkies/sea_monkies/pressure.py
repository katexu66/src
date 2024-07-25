#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import Altitude
from sensor_msgs.msg import FluidPressure as Pressure

class pressure_node(Node):

    def __init__(self):
        super().__init__("pressure_node")

        self.pressure = 0

        self.sub = self.create_subscription(
            Pressure,
            "bluerov2/pressure",
            self.pressure_callback,
            10
        )

        self.depth_pub = self.create_publisher(
            Altitude,
            "bluerov2/depth",
            10
        )

        self.get_logger().info("starting subscriber nodes")

    def depth_calculation(self):
        # p = p*g*h
        gravity = 9.81 # m/s2
        density = 1000 # kg/m3
        self.depth = self.pressure/(gravity*density)
        msg = Altitude()
        msg.local = self.depth
        self.publisher.publish(msg)

    def pressure_callback(self, msg):
        self.get_logger().info(f"Pressure: {msg.fluid_pressure}")
        self.pressure = msg.fluid_pressure
        self.depth_calculation()

    def destroy_node(self):
        return super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = pressure_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__=="__main__":
    main()