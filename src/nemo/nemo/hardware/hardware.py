import rclpy
from rclpy.node import Node

class Hardware(Node):
    def __init__(self):
        super().__init__('hardware')

        self.get_logger().info("")

def main(args=None):
    rclpy.init(args=args)

    node = Hardware()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        pass
    finally:
        node.shutdown()
        rclpy.try_shutdown()
        node.destroy_node()