import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from slamware_ros_sdk.msg import GoHomeRequest, AddLineRequest, ClearMapRequest, RemoveLineRequest, Line2DFlt32Array
from geometry_msgs.msg import Twist, PoseStamped, Pose


import time, math

class Head_example(Node):
    def __init__(self):
        super().__init__('slam_node')
        
        self.topic_sub_group = MutuallyExclusiveCallbackGroup()
        
        TOPIC_SUBS = {
            'robot_pose': ('/robot_pose', PoseStamped,self.robot_pose_callback),
            'virtual_wall':('/virtual_walls',Line2DFlt32Array,self.virtual_wall_callback)
        }

        # 토픽 구독 설정
        self.topic_subs = {}
        for topic_tag, (topic_name, topic_type,topic_callback_fun) in TOPIC_SUBS.items():
            self.topic_subs[topic_tag] = self.create_subscription(topic_type,topic_name,topic_callback_fun,10,callback_group=self.topic_sub_group)
            self.get_logger().info(f"Subscriber created: {topic_tag} -> {topic_name} with {topic_callback_fun}")
            
        TOPIC_PUBS = {
            'cmd_vel':('/cmd_vel',Twist),
            'goal_pose':('/move_base_simple/goal',PoseStamped),
            'set_pose':('/slamware_ros_sdk_server_node/set_pose',Pose),
            'go_home':('/slamware_ros_sdk_server_node/go_home',GoHomeRequest),
            'add_line':('/slamware_ros_sdk_server_node/add_line',AddLineRequest),
            'remove_line':('/slamware_ros_sdk_server_node/remove_line',RemoveLineRequest),
            'clear_map':('/slamware_ros_sdk_server_node/clear_map',ClearMapRequest),
        }
        
        # 퍼블리셔 등록
        self.topic_pubs = {}
        for topic_tag, (topic_name, topic_type) in TOPIC_PUBS.items():
            self.topic_pubs[topic_tag] = self.create_publisher(topic_type, topic_name, 10)
            self.get_logger().info(f"Publisher created: {topic_tag} -> {topic_name}")
            


    def robot_pose_callback(self, msg: PoseStamped):
        # 위치
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # 쿼터니언
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w


        # 로그 출력 (또는 변수 저장)
        #self.get_logger().info(f"[Robot Pose] Position: x={x:.2f}, y={y:.2f}, z={z:.2f} | qx={qx:.2f} qy={qy:.2f}, qz={qz:.2f}, qw={qw:.2f}")
        
        
    def virtual_wall_callback(self, msg):
        for line in msg.lines:
            line_id = line.id  # 각 선의 ID
            start = line.start
            end = line.end

            self.get_logger().info(
                f"[Virtual Wall] ID: {line_id}, "
                f"Start: ({start.x:.2f}, {start.y:.2f}), "
                f"End: ({end.x:.2f}, {end.y:.2f})"
            )

    def robot_homing(self):
        msg=GoHomeRequest()
        self.topic_pubs['go_home'].publish(msg)
        
    def task(self):
        self.robot_homing()

def main(args=None):
    rclpy.init(args=args)
    node = Head_example()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    executor.create_task(node.task)
    try:
        node.get_logger().info("✅ Master Server is running...")
        executor.spin()  # ✅ 멀티스레드 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
