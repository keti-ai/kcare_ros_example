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
        
    def move(self, linear_x=0.0, angular_z=0.0, duration=2.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z

        start_time = self.get_clock().now().seconds_nanoseconds()[0]

        

        while rclpy.ok():
            now = self.get_clock().now().seconds_nanoseconds()[0]
            if now - start_time >= duration:
                break
            self.topic_pubs['cmd_vel'].publish(msg)
            self.get_logger().info(f"Publishing: linear.x = {linear_x:.2f}, angular.z = {angular_z:.2f}")
            time.sleep(0.1)

        # 정지 명령
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        self.topic_pubs['cmd_vel'].publish(msg)
        self.get_logger().info("Stopped.")

    def publish_goal_pose(self,point_x,point_y,target_yaw):
        goal = PoseStamped()
        goal.header.frame_id = "map"  # 또는 "odom" 등 사용 중인 좌표계에 따라 다름
        goal.header.stamp = self.get_clock().now().to_msg()

        # 목표 위치 설정 (예: x=2.0, y=3.0)
        goal.pose.position.x = point_x
        goal.pose.position.y = point_y
        goal.pose.position.z = 0.0

        # 방향 (yaw = 90도 = pi/2 rad)
        yaw = math.pi / 2
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.publisher_.publish(goal)
        self.get_logger().info(f"📍 Goal published to (x={goal.pose.position.x}, y={goal.pose.position.y}, yaw={yaw:.2f})")


    def publish_set_pose(self):
        pose_msg = Pose()

        # 위치 설정
        pose_msg.position = Point(x=1.0, y=2.0, z=0.0)

        # 방향 설정 (yaw = 90도 = pi/2 rad)
        yaw = math.pi / 2
        pose_msg.orientation = Quaternion(
            x=0.0,
            y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )

        self.publisher_.publish(pose_msg)
        self.get_logger().info(f"📍 Set pose published: x=1.0, y=2.0, yaw={yaw:.2f} rad")


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
