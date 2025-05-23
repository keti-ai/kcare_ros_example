import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from kcare_robot_ros2_controller_msgs.msg import HeadState
from kcare_robot_ros2_controller_msgs.srv import HeadPoseCommand

import time, math

class RobotParam:
    spin_time: float = 0.05     # ROS2 루프문 대기시간
    elev_home: float = 200.0      # 리프트 홈위치
    arm_home: list = [math.radians(90.0),0.0,0.0,0.0,0.0,0.0,math.radians(-90.0)]   # 조인트좌표계 로봇 홈자세
    arm_ready: list = [math.radians(90.0),math.radians(15.0),0.0,math.radians(15.0),0.0,math.radians(-90.0),0.0]    #조인트좌표계 로봇 준비자세
    arm_giving: list = [math.radians(130.0),math.radians(15.0),0.0,math.radians(15.0),0.0,math.radians(-90.0),0.0]
    j_arm_speed: float = 0.6    # 조인트좌표계 로봇 속도
    j_arm_accel: float = 10.0   # 조인트좌표계 로봇 가속도
    l_arm_speed: float = 200.0      # 베이스좌표계 로봇 속도
    l_arm_accel: float = 1000.0     # 베이스좌표계 로봇 가속도
    grip_open: int = 1000       # 그리퍼 오픈 상태 퍼센트
    grip_close: int = 0     #그리퍼 닫음 상태 퍼샌트
    grip_max_force: int = 100   # 그리퍼 최대힘
    grip_min_force: int = 50    # 그리퍼 최소힘

class Head_example(Node):
    def __init__(self):
        super().__init__('head_node')
        
        # Xarm Service clients
        SERVICE_CLIENTS ={
            'head_command' : ('/head/pose_command',HeadPoseCommand),
        }
        
        self.service_clients = {}
        for service_tag, (service_name, service_type) in SERVICE_CLIENTS.items():
            self.service_clients[service_tag] = self.create_client(service_type, service_name)
            self.get_logger().info(f"Service Client created: {service_tag} -> {service_name}")

        TOPIC_SUBS = {
            'head_state': ('/head/state', HeadState,self.head_state_callback),
        }
        
        self.topic_sub_group = MutuallyExclusiveCallbackGroup()
        
        # 토픽 구독 설정
        self.topic_subs = {}
        for topic_tag, (topic_name, topic_type,topic_callback_fun) in TOPIC_SUBS.items():
            self.topic_subs[topic_tag] = self.create_subscription(topic_type,topic_name,topic_callback_fun,10,callback_group=self.topic_sub_group)
            self.get_logger().info(f"Subscriber created: {topic_tag} -> {topic_name} with {topic_callback_fun}")
            
    def head_state_callback(self,msg):
        self.head_state=[msg.current_rz,msg.current_ry]
        self.get_logger().info(f"head_pose callback: {self.head_state}")
        
    def call_head_command(self,head_pose,wait=True):
        request=HeadPoseCommand.Request()
        request.rz = head_pose[0]
        request.ry = head_pose[1]
        request.wait = wait

        future = self.service_clients['head_command'].call_async(request)

        if not wait:
            return
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Head Move : {response.successed}')
        else:
            self.get_logger().error('Failed to call service /gripper/command')
            
    def rb_init(self):
        # Head 지령 Z축으로 20도. 이동 완료시 다음코드 실행
        self.call_head_command([0.0, 20.0])
        # Head 지령 Z축 20도, Y축 20도. wait false의 경우 완료대기없이 바로 다음 코드 실행
        self.call_head_command([20.0,20.0],wait=False)
        time.sleep(2)
        # Head 지령 Z축 -40도
        self.call_head_command([0.0, -40.0])

def main(args=None):
    rclpy.init(args=args)
    node = Head_example()
    
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    # 노드내 함수 비동기 실행.초기화 함수등 실행
    executor.create_task(node.rb_init)
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
