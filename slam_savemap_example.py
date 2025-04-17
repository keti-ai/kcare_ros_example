import rclpy
from rclpy.node import Node

from slamware_ros_sdk.srv import SyncGetStcm


class StcmMapSaver(Node):
    def __init__(self):
        super().__init__('stcm_map_saver')
        self.cli = self.create_client(SyncGetStcm, '/sync_get_stcm')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for sync_get_stcm service...')
        self.req = SyncGetStcm.Request()

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            with open("saved_map.stcm", "wb") as f:
                f.write(future.result().raw_stcm)
            self.get_logger().info("Map saved to saved_map.stcm")
        else:
            self.get_logger().error("Failed to receive map data")

def main(args=None):
    rclpy.init(args=args)
    node = StcmMapSaver()
    node.send_request()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()