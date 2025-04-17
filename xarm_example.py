import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from xarm_msgs.msg import RobotMsg
#For Robot arm Control
from xarm_msgs.srv import MoveJoint, MoveCartesian, SetInt16, SetInt16ById, Call
#For Robot tool Control
from xarm_msgs.srv import SetInt32, SetModbusTimeout, GetSetModbusData

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

class Xarm_example(Node):
    def __init__(self):
        super().__init__('xarm_node')
        
        # Xarm Service clients
        SERVICE_CLIENTS ={
            'set_servo_angle' : ('/xarm/set_servo_angle',MoveJoint),
            'set_servo_move': ('/xarm/set_position', MoveCartesian),
            'set_servo_tool': ('/xarm/set_tool_position',MoveCartesian),
            'motion_enable': ('/xarm/motion_enable', SetInt16ById),
            'set_mode': ('/xarm/set_mode', SetInt16),
            'set_state': ('/xarm/set_state', SetInt16),
            'clean_error': ('/xarm/clean_error', Call),
            'set_xarm_tool_baud' : ('/xarm/set_tgpio_modbus_baudrate',SetInt32),
            'set_xarm_tool_timeout' : ('/xarm/set_tgpio_modbus_timeout',SetModbusTimeout),
            'getset_xarm_modbus_data' : ('/xarm/getset_tgpio_modbus_data',GetSetModbusData),
        }
        
        self.service_clients = {}
        for service_tag, (service_name, service_type) in SERVICE_CLIENTS.items():
            self.service_clients[service_tag] = self.create_client(service_type, service_name)
            self.get_logger().info(f"Service Client created: {service_tag} -> {service_name}")

        # 로봇 토픽 구독독
        TOPIC_SUBS = {
            'robot_pose': ('/xarm/robot_states', RobotMsg,self.robot_pose_callback),
        }
        
        self.topic_sub_group = MutuallyExclusiveCallbackGroup()
        
        # 토픽 구독 설정
        self.topic_subs = {}
        for topic_tag, (topic_name, topic_type,topic_callback_fun) in TOPIC_SUBS.items():
            self.topic_subs[topic_tag] = self.create_subscription(topic_type,topic_name,topic_callback_fun,10,callback_group=self.topic_sub_group)
            self.get_logger().info(f"Subscriber created: {topic_tag} -> {topic_name} with {topic_callback_fun}")
       
        
    def robot_pose_callback(self, msg):
        # 로봇팔 제어상태, 모드, 에러코드, 각도, position값값  
        self.get_logger().info(f'Robot State: "%s"' % str(msg.state))
        self.get_logger().info(f'Robot Mode: "%s"' % str(msg.mode))
        self.get_logger().info(f'Robot Error: "%s"' % str(msg.err))
        self.get_logger().info(f'Robot Angles: "%s"' % str(msg.angle))
        self.get_logger().info(f'Robot Pose: "%s"' % str(msg.pose))
        
        
    def call_set_mode(self, mode=0, wait=True):
        # 모드 정의 관련 서비스 호출 예시
        '''
        Mode 0 : xArm controller (Position) mode.
        Mode 1 : External trajectory planner (position) mode.
        Mode 2 : Free-Drive (zero gravity) mode.
        Mode 3 : Reserved.
        Mode 4 : Joint velocity control mode.
        Mode 5 : Cartesian velocity control mode.
        Mode 6 : Joint space online planning mode. (Firmware >= v1.10.0)
        Mode 7 : Cartesian space online planning mode. (Firmware >= v1.11.0)
        '''
        request = SetInt16.Request()
        request.data = mode  # 설정할 모드

        # 서비스 호출
        future = self.service_clients['set_mode'].call_async(request)
        
        if not wait:
            return
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Mode set to: {mode}, Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_mode')
    
    def call_set_state(self, state=0):
        # 로봇 스테이트 관련 서비스 호출 예시
        '''
        state:
        0: motion state
        3: pause state
        4: stop state
        '''
        request = SetInt16.Request()
        request.data = state  # 설정할 상태

        # 서비스 호출
        future = self.service_clients['set_state'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'State set to: {state}, Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_state')

    def call_motion_enable(self, id=8, data=1):
        # 로봇 토크 활성화
        request = SetInt16ById.Request()
        request.id = id
        request.data = data
        
        # 서비스 호출
        future = self.service_clients['motion_enable'].call_async(request)

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Motion Enabled Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/motion_enable')

    def call_clean_error(self):
        # 로봇 에러상태 클린
        request = Call.Request()
        
        future = self.service_clients['clean_error'].call_async(request)
        
        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Clean Error Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/clean_error')

    def call_set_servo_angle(self, angle,speed = RobotParam.j_arm_speed, acc=RobotParam.j_arm_accel, wait=True):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveJoint.Request()
        
        # 홈 위치 각도 설정
        request.angles = angle 
        request.speed = speed  # 속도 설정
        request.acc = acc    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        # 서비스 호출
        future = self.service_clients['set_servo_angle'].call_async(request)
        
        if not wait:
            return

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Joint Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_servo_angle')


    def call_set_servo_move(self, pose, relative=False,mvtype=0,wait=True):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveCartesian.Request()
        # 홈 위치 각도 설정
        request.pose = pose 
        request.speed = RobotParam.l_arm_speed  # 속도 설정
        request.acc = RobotParam.l_arm_accel    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        request.relative = relative
        request.motion_type = mvtype

        # 서비스 호출
        future = self.service_clients['set_servo_move'].call_async(request)
        
        if not wait:
            return

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Move Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_position')
        
    def call_set_tool_move(self, pose, relative=False,mvtype=0,wait=True):
        # 홈 위치로 서보 각도 설정 서비스 호출 예시
        request = MoveCartesian.Request()
        # 홈 위치 각도 설정
        request.pose = pose 
        request.speed = RobotParam.l_arm_speed  # 속도 설정
        request.acc = RobotParam.l_arm_accel    # 가속도 설정
        request.mvtime = 0.0    # 이동 시간 설정
        request.wait = wait    # 완료 대기 설정
        request.relative = relative
        request.motion_type = mvtype
        
        # 서비스 호출
        future = self.service_clients['set_servo_tool'].call_async(request)
        
        if not wait:
            return

        # 비동기 호출의 결과를 기다림
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Servo Move Response: {response.ret}, Message: {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_tool_position')
        
        
    def call_set_relative_robot_pose(self,dx=0.0,dy=0.0,dz=0.0,rdx=0.0,rdy=0.0,rdz=0.0,wait=True):
        target_pose=[dx,dy,dz,rdx,rdy,rdz]
        self.call_set_servo_move(target_pose,relative=True,wait=wait)
        
    def call_set_relative_tool_pose(self,dx=0.0,dy=0.0,dz=0.0,rdx=0.0,rdy=0.0,rdz=0.0,wait=True):
        target_pose=[dx,dy,dz,rdx,rdy,rdz]
        self.call_set_tool_move(target_pose,relative=True,wait=wait)
        
    def xarm_set_tool_baudrate(self,baudrate=115200):
        request=SetInt32.Request()
        request.data=baudrate

        future = self.service_clients['set_xarm_tool_baud'].call_async(request)
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
            
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Xarm set Tool Baud : {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_tgpio_modbus_baudrate') 
            

    def xarm_set_tool_timeout(self,timeout=30):
        request=SetModbusTimeout.Request()
        request.timeout=timeout

        future = self.service_clients['set_xarm_tool_timeout'].call_async(request)
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)
            
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Xarm set Tool Timeout : {response.message}')
        else:
            self.get_logger().error('Failed to call service /xarm/set_tgpio_modbus_timeout') 


    def xarm_modbus_data(self,data,wait=True):
        request=GetSetModbusData.Request()
        request.modbus_data=data
        request.modbus_length=len(data)
        request.ret_length=11
        
        future = self.service_clients['getset_xarm_modbus_data'].call_async(request)
        
        if not wait:
            return None
        
        while not future.done():
            # 여기서는 별도로 spin_once가 이미 다른 스레드에서 처리되므로
            # 추가적으로 spin_once(self)를 호출하지 않습니다.
            time.sleep(RobotParam.spin_time)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Xarm Send Modbus Result Return : {response.ret_data}')
            return response.ret_data
        else:
            self.get_logger().error('Failed to call service getset_xarm_modbus_data')
            return None
         
    def xarm_gripper_init(self):
        data = [0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x65, 0x00, 0x00]
        return self.xarm_modbus_data(data)
        
    def xarm_set_motor_torque(self, ratio):
        data = [
            0x01,
            0x10,
            0x00, 0x00,
            0x00, 0x02,
            0x04,
            0x00, 0xD4,  # Command 212
            (ratio >> 8) & 0xFF, ratio & 0xFF
        ]
        return self.xarm_modbus_data(data)

    def xarm_set_finger_position(self, position):
        data = [
            0x01,       # slave ID
            0x10,       # Function code: Write Multiple Registers
            0x00, 0x00, # Start address: Register 0
            0x00, 0x02, # Register count: 2
            0x04,       # Byte count: 4 bytes (2 registers)
            0x00, 0x68, # Register 0: Command 104 (Set Finger Position)
            (position >> 8) & 0xFF, position & 0xFF  # Register 1: Position
        ]
        return self.xarm_modbus_data(data,wait=False)

    def xarm_set_gripper_speed_percent(self, percent):
        data = [
            0x01,       # slave ID
            0x10,       # function code: Write Multiple Registers
            0x00, 0x00, # start address: register 0
            0x00, 0x02, # number of registers to write
            0x04,       # byte count (2 regs × 2 bytes)
 행
        self.call_set_servo_angle(RobotParam.arm_home)
        self.call_set_servo_angle(RobotParam.arm_ready)

        # Base좌표계 기준 Z축으로 -30도 회전
        self.call_set_relative_robot_pose(rdz=math.radians(-30.0))
        # Tool 좌표계 기준 Z축으로 50mm 이동
        self.call_set_relative_tool_pose(dz=50.0)
        # 그리퍼 툴 닫기
        self.xarm_set_finger_position(RobotParam.grip_close)
        # 그리퍼 대기
        time.sleep(3)
        # Tool 좌표계 기준 X축 50mm 이동
        self.call_set_relative_tool_pose(dx=50.0)
        # 조인트 좌표계 준비자세 이동. 이동완료 대기없이 다음 동작 곧바로 실행
        self.call_set_servo_angle(RobotParam.arm_ready, wait=False)
        # 그리퍼 열기
        self.xarm_set_finger_position(RobotParam.grip_open)
        # 그리퍼 대기
        time.sleep(3)
        # 조인트좌표계 홈자세 이동
        self.call_set_servo_angle(RobotParam.arm_home)
        



def main(args=None):
    rclpy.init(args=args)
    node = Xarm_example()
    
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
