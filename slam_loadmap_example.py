# load_stcm_with_pose.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from slamware_ros_sdk.srv import SyncSetStcm

class StcmMapLoaderWithPose(Node):
    def __init__(self):
        super().__init__('stcm_map_loader_with_pose')
        self.cli = self.create_client(SyncSetStcm, '/sync_set_stcm')
        while not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for sync_set_stcm service...')
        self.req = SyncSetStcm.Request()

    def send_request(self, filename):
        # Load .stcm map file
        try:
            with open(filename, "rb") as f:
                self.req.raw_stcm = f.read()
        except Exception as e:
            self.get_logger().error(f"Failed to read file: {e}")
            return

        # Set initial pose to (0.0, 0.0, yaw=0.0)
        self.req.robot_pose = Pose()
        self.req.robot_pose.position = Point(x=0.0, y=0.0, z=0.0)
        # Convert yaw to quaternion (yaw = 0.0 means facing x-direction)
        self.req.robot_pose.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=0.0,
            w=1.0
        )

        # Send request
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info("Map and initial pose successfully set")
        else:
            self.get_logger().error("Failed to load map with pose")

def main(args=None):
    rclpy.init(args=args)
    node = StcmMapLoaderWithPose()
    node.send_request("saved_map.stcm")  # 불러올 맵 경로
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
