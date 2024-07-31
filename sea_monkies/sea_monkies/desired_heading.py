class TutorialPublisher(Node):
    def __init__(self):
        super().__init__("tutorial_publisher")
        self.publisher = self.create_publisher(
            OverrideRCIn,
            "bluerov2/override_rc",
            10
        )
        self.publisher_timer = self.create_timer(40.0, self.run_node)
        self.get_logger().info("starting publisher node")

    def desired_heading(self):
        msg = OverrideRCIn()
        msg.channels[0] = 1500
        msg.channels[1] = 1500
        msg.channels[2] = 1500 # roll
        msg.channels[3] = 1500 # up/down
        msg.channels[4] = 2000 # rotate
        msg.channels[5] = 1500 # forward
        msg.channels[6] = 1500 # backwards
        msg.channels[7] = 1500 # strafe
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TutorialPublisher()

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