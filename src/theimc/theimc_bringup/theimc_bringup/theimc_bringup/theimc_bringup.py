#!/usr/bin/env python3
#
# Copyright 2023 EduRobotAILab CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Leo Cho
# Modified by Gemini for customization (Arduino integration)

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, Pose, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import serial
import time
import sys # sys 모듈 추가 (시리얼 포트 연결 실패 시 종료용)

# --- 데이터 구조 클래스 정의 ---
class OdomPose(object):
    """
    오도메트리 기반 로봇의 현재 위치 및 방향을 저장하는 클래스.
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    
class Encoder(object):
    """
    엔코더 데이터를 처리하는 클래스.
    current_data: 현재 엔코더 값 (아두이노에서 수신)
    """
    current_data = 0.0 # 아두이노에서 받은 절대 펄스 값
    # Python에서 delta_data와 distance는 직접 사용하지 않고, 아두이노에서 계산된 속도/거리 사용

class CmdVel():
    """
    cmd_vel (선속도, 각속도) 명령을 저장하는 클래스.
    lin_vel_x: 선형 속도 (x축)
    ang_vel_z: 각속도 (z축)
    """
    lin_vel_x = 0.0
    ang_vel_z = 0.0

class Joint(object):
    """
    로봇 조인트 (휠)의 상태를 저장하는 클래스.
    joint_name: 조인트 이름 리스트
    joint_pos: 조인트 현재 위치 (각도)
    joint_vel: 조인트 현재 속도 (각속도)
    """
    joint_name = ['wheel_left_joint', 'wheel_right_joint']
    joint_pos = [0.0, 0.0]
    joint_vel = [0.0, 0.0]

class BringUp(Node):
    """
    로봇의 하드웨어 인터페이스를 위한 ROS 2 브링업 노드.
    시리얼 통신을 통해 엔코더 데이터를 읽고 모터를 제어하며,
    ROS 2 토픽으로 오도메트리, 조인트 상태를 발행합니다.
    """
    def __init__(self):
        super().__init__('bring_up')
        
        # --- 시리얼 포트 설정 ---
        # 아두이노 연결에 맞춰 포트와 보드레이트를 설정합니다.
        self.serial_port = "/dev/ttyUSB1" # - ttyUSB1 사용
        self.baud_rate = 9600             # - 아두이노 Serial.begin(9600)에 맞춤
        self.serial_timeout = 0.1         # 시리얼 통신 타임아웃 (짧게 설정하여 블로킹 방지)

        try:
            self.pico_serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout
            )
            self.pico_serial.reset_input_buffer()  # 입력 버퍼 초기화
            self.pico_serial.reset_output_buffer() # 출력 버퍼 초기화
            self.get_logger().info(f"Successfully connected to serial port: {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            self.get_logger().error("Please check if the device is connected and permissions are set correctly (e.g., 'sudo usermod -a -G dialout $USER').")
            rclpy.shutdown() # 시리얼 포트 연결 실패 시 노드 종료
            sys.exit(1)
        
        # --- 로봇 물리적 파라미터 ---

        # 아두이노 코드 (WHEEL_DISTANCE_MM = 780, WHEEL_RADIUS_MM = 65.0, ENCODER_RESOLUTION = 1800) 반영
        self.wheel_separation = 0.780  # [m] 좌우 바퀴 간의 거리 (780mm)
        self.wheel_radius = 0.065      # [m] 바퀴의 반지름 (65mm)
        self.enc_resolution = 1800     # 엔코더 1회전당 펄스 수 (아두이노의 ENCODER_RESOLUTION)
        # 펄스당 미터 변환 계수 계산: 아두이노 코드의 DISTANCE_PER_PULSE_MM (mm/pulse)을 역산하여 m/pulse로 변환
        self.pulse_per_meter = self.enc_resolution / (2 * math.pi * self.wheel_radius)
        
        # NOTE: 시리얼 통신 지연시간은 Python에서 명령 전송 후 Readline을 할 때 의미가 있었으나,
        # 아두이노가 비동기적으로 데이터를 전송하므로 여기서는 크게 중요하지 않습니다.
        # 그러나 안전을 위해 메시지 간 약간의 대기 시간을 유지합니다.
        self.serial_write_delay = 0.01 # [s] 시리얼 명령 전송 후 대기 시간

        # --- 속도 제한 ---
        # ROS 2 cmd_vel 입력에 대한 내부적인 최대 속도 제한
        # 이 값들은 아두이노의 속도-PWM 매핑의 최대치와 일치해야 합니다.
        self.max_lin_vel_x = 0.5 # [m/s] 최대 선형 속도 (아두이노에서 이 속도가 최대 PWM에 매핑됩니다)
        self.max_ang_vel_z = 0.5 # [rad/s] 최대 각속도
        
        self.timestamp_previous = self.get_clock().now() # 이전 타임스탬프 (dt 계산용)
        
        # 인스턴스 초기화
        self.enc_r = Encoder()
        self.enc_l = Encoder()
        self.odom_pose = OdomPose()
        self.joint = Joint()          
        self.cmd_vel = CmdVel()       

        # QoS 프로파일 설정 (안정적인 통신을 위해 depth, reliability, history policy 설정)
        qos_profile = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ROS 2 Subscriber 설정
        # `cmd_vel` 토픽으로부터 로봇의 이동 명령을 받습니다.
        self.sub_cmd_vel_msg = self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel_msg, qos_profile)
        self.get_logger().info("Subscribing to /cmd_vel topic.")

        # ROS 2 Publisher 설정
        # `joint_states`: 로봇 휠의 조인트 상태 (위치, 속도)를 발행합니다.
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', qos_profile)
        # `odom`: 로봇의 오도메트리 정보 (위치, 속도)를 발행합니다.
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos_profile)
        # `odom_tf`: 오도메트리 프레임(odom)과 로봇 베이스 프레임(base_footprint) 간의 TF(Transform)를 발행합니다.
        self.pub_odom_tf = TransformBroadcaster(self)
        # NOTE: 아두이노 코드에서 IMU 데이터(롤, 피치, 요, 각속도)를 직접 전송하지 않으므로,
        # Pose 발행 토픽은 현재 비활성화하거나 제거합니다. 필요시 아두이노 코드 수정 후 재활성화하세요.
        # self.pub_pose = self.create_publisher(Pose, 'pose', qos_profile)
        self.get_logger().info("Publishers for /joint_states and /odom created. Pose publisher disabled due to no IMU data from Arduino.")

        # 타이머 설정
        # 아두이노의 데이터 전송 주기 (50ms)와 유사하게 0.05초마다 `update_robot` 함수를 호출합니다.
        self.timer_period = 0.05 # [s] 업데이트 주기
        self.timer_proc = self.create_timer(self.timer_period, self.update_robot)
        self.get_logger().info(f"Robot update timer set to {self.timer_period} seconds.")
        
        self.last_sent_cmd_vel_x = 0.0 # 마지막으로 전송한 cmd_vel.linear.x 값
        self.last_sent_cmd_vel_z = 0.0 # 마지막으로 전송한 cmd_vel.angular.z 값
              
    def cb_cmd_vel_msg(self, cmd_vel_msg):    
        """
        cmd_vel 토픽 메시지를 수신하여 로봇의 선형 및 각속도를 설정하고,
        해당 속도에 맞춰 아두이노가 이해하는 연속적인 속도 명령을 시리얼로 전송합니다.
        """
        current_lin_vel_x = cmd_vel_msg.linear.x  # [m/s]
        current_ang_vel_z = cmd_vel_msg.angular.z # [rad/s]
        
        # ROS 2 cmd_vel 입력에 대한 민감도를 조절하는 데드존입니다.
        # 아주 작은 속도 변화는 무시하고 정지 상태를 유지합니다.
        linear_deadzone = 0.02 # [m/s]
        angular_deadzone = 0.03 # [rad/s]

        # 데드존 적용
        if abs(current_lin_vel_x) < linear_deadzone:
            current_lin_vel_x = 0.0
        if abs(current_ang_vel_z) < angular_deadzone:
            current_ang_vel_z = 0.0

        # 최대 속도 제한
        # 아두이노에서 매핑할 최대 속도를 벗어나지 않도록 클램핑합니다.
        current_lin_vel_x = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, current_lin_vel_x))
        current_ang_vel_z = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, current_ang_vel_z))

        # 이전에 보낸 명령과 동일하면 다시 보내지 않아 시리얼 트래픽 감소
        # 부동 소수점 비교이므로 약간의 오차 허용
        if (abs(current_lin_vel_x - self.last_sent_cmd_vel_x) > 0.001 or
            abs(current_ang_vel_z - self.last_sent_cmd_vel_z) > 0.001):
            
            try:
                # "ROS_CMD,<linear_x>,<angular_z>\n" 형식으로 아두이노에 명령 전송
                command_string = f"ROS_CMD,{current_lin_vel_x:.3f},{current_ang_vel_z:.3f}\n"
                self.pico_serial.write(command_string.encode("utf-8"))
                time.sleep(self.serial_write_delay) 
                
                self.last_sent_cmd_vel_x = current_lin_vel_x
                self.last_sent_cmd_vel_z = current_ang_vel_z
                # self.get_logger().info(f"Sent: {command_string.strip()}") # 디버깅용
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error during motor command: {e}")
                self._send_stop_commands() # 오류 발생 시 정지 명령 전송

    def _send_stop_commands(self):
        """
        모터 정지 명령을 시리얼로 전송합니다.
        """
        try:
            # 아두이노에 0,0 속도 명령 전송
            self.pico_serial.write(b"ROS_CMD,0.000,0.000\n") 
            time.sleep(self.serial_write_delay)
            self.last_sent_cmd_vel_x = 0.0
            self.last_sent_cmd_vel_z = 0.0
            self.get_logger().info("Sent motor stop command 'ROS_CMD,0.000,0.000'.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send stop commands: {e}")
   
    def update_robot(self):
        """
        타이머 콜백 함수: 아두이노에서 주기적으로 전송되는 엔코더 데이터를 읽고,
        오도메트리, 조인트 상태를 계산하여 ROS 2 토픽으로 발행합니다.
        """
        if not self.pico_serial.is_open:
            self.get_logger().warn("Serial port is not open. Skipping robot update.")
            return

        # 시간 간격 (dt) 계산
        self.timestamp_now = self.get_clock().now()
        dt = (self.timestamp_now - self.timestamp_previous).nanoseconds * 1e-9
        if dt == 0: # dt가 0인 경우 (매우 드물지만) 오류 방지
            return
        self.timestamp_previous = self.timestamp_now
        
        # --- 엔코더 데이터 읽기 및 파싱 ---
        # 아두이노는 "ENC,count1,count2,vel1,vel2" 형식으로 주기적으로 데이터를 전송합니다.
        # 시리얼 버퍼에서 가용한 모든 라인을 읽어 가장 최신 데이터를 파싱합니다.
        latest_encoder_data = None
        while self.pico_serial.in_waiting > 0:
            try:
                line = self.pico_serial.readline().decode('utf-8').strip()
                if line.startswith("ENC,"):
                    latest_encoder_data = line
            except UnicodeDecodeError:
                # 가끔 깨진 데이터가 들어올 수 있으므로 무시
                pass
            except serial.SerialException as e:
                self.get_logger().error(f"Serial read error: {e}")
                break # 오류 발생 시 루프 종료

        if latest_encoder_data:
            try:
                parts = latest_encoder_data.split(',')
                if len(parts) == 5 and parts[0] == "ENC":
                    # 아두이노에서 보낸 데이터 파싱
                    # count1: 왼쪽 엔코더 절대 펄스 수
                    # count2: 오른쪽 엔코더 절대 펄스 수
                    # vel1: 왼쪽 엔코더 속도 (펄스/초)
                    # vel2: 오른쪽 엔코더 속도 (펄스/초)
                    left_enc_count = float(parts[1])
                    right_enc_count = float(parts[2])
                    left_enc_vel_pps = float(parts[3])
                    right_enc_vel_pps = float(parts[4])

                    # 엔코더 절대 위치 업데이트 (조인트 상태에 사용)
                    self.enc_l.current_data = left_enc_count
                    self.enc_r.current_data = right_enc_count

                    # 엔코더 속도 (펄스/초)를 미터/초로 변환
                    lin_vel_left_mps = left_enc_vel_pps / self.pulse_per_meter
                    lin_vel_right_mps = right_enc_vel_pps / self.pulse_per_meter

                    # 로봇의 선형 속도 (trans_vel) 및 각속도 (orient_vel) 계산
                    trans_vel = (lin_vel_left_mps + lin_vel_right_mps) / 2.0
                    orient_vel = (lin_vel_right_mps - lin_vel_left_mps) / self.wheel_separation

                    # 오도메트리 및 조인트 상태 업데이트 함수 호출
                    self.update_odometry(trans_vel, orient_vel, dt)
                    self.update_JointStates(left_enc_count, right_enc_count, trans_vel, orient_vel)
                else:
                    self.get_logger().warn(f"Received malformed encoder data: {latest_encoder_data}")
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error parsing encoder data '{latest_encoder_data}': {e}")
        # else:
            # self.get_logger().warn("No valid encoder data received.") # 데이터가 항상 올 필요는 없으므로 주석 처리

    def update_odometry(self, trans_vel, orient_vel, dt):
        """
        오도메트리 데이터를 계산하고 ROS 2 `Odometry` 토픽 및 TF를 발행합니다.
        """
        # 현재 로봇의 자세 (theta) 업데이트
        self.odom_pose.theta += orient_vel * dt # [rad]

        # 로봇의 위치 (x, y) 업데이트
        # 선형 속도를 현재 방향에 따라 X, Y 축으로 분해하여 이동 거리를 계산
        d_x = trans_vel * math.cos(self.odom_pose.theta)
        d_y = trans_vel * math.sin(self.odom_pose.theta)
        self.odom_pose.x += d_x * dt  # [m]
        self.odom_pose.y += d_y * dt  # [m]
        
        # 오일러 각도(yaw)를 쿼터니언으로 변환
        q = quaternion_from_euler(0, 0, self.odom_pose.theta) # roll, pitch는 0으로 가정 (2D 평면 이동)
        
        # Odometry 메시지 발행
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"        
        odom.header.stamp = self.timestamp_now.to_msg()
        
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0 # 2D 로봇이므로 Z는 항상 0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = trans_vel
        odom.twist.twist.linear.y = 0.0 # 2D 로봇이므로 Y 선형 속도는 0
        odom.twist.twist.angular.z = orient_vel # Z축 각속도

        self.pub_odom.publish(odom)

        # TF(Transform) 메시지 발행 (odom -> base_footprint)
        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id        
        odom_tf.header.stamp = self.timestamp_now.to_msg()
        
        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = odom.pose.pose.position.z
        odom_tf.transform.rotation = odom.pose.pose.orientation
        self.pub_odom_tf.sendTransform(odom_tf)

    def update_JointStates(self, odo_l_count, odo_r_count, trans_vel, orient_vel):
        """
        조인트 상태 데이터를 계산하고 ROS 2 `JointState` 토픽을 발행합니다.
        """
        # 엔코더 펄스 수를 바퀴의 각도 (라디안)로 변환
        # 아두이노에서 보낸 odo_l_count, odo_r_count는 현재 엔코더의 절대 펄스 수입니다.
        wheel_ang_left = (odo_l_count / self.enc_resolution) * (2 * math.pi)
        wheel_ang_right = (odo_r_count / self.enc_resolution) * (2 * math.pi)

        # 바퀴의 각속도 (rad/s) 계산
        # 로봇의 선형/각속도를 이용하여 각 바퀴의 각속도를 역산
        wheel_ang_vel_left = (trans_vel - (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius    
        wheel_ang_vel_right = (trans_vel + (self.wheel_separation / 2.0) * orient_vel) / self.wheel_radius  

        self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]
        
        # JointState 메시지 발행
        joint_states = JointState()
        joint_states.header.frame_id = "base_link" # 로봇의 베이스 링크
        joint_states.header.stamp = self.timestamp_now.to_msg()
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = [] # 노력(Effort)은 사용하지 않으므로 비워둡니다.
        self.pub_joint_states.publish(joint_states)
    
    # NOTE: 아두이노 코드에서 IMU 데이터(roll, pitch, yaw, vel_z)를 전송하지 않으므로,
    # update_PoseStates 함수는 더 이상 필요하지 않습니다. 주석 처리하거나 제거합니다.
    # def update_PoseStates(self, roll, pitch, yaw, vel_z):
    #     pose = Pose()
    #     pose.orientation.x = roll
    #     pose.orientation.y = pitch
    #     pose.orientation.z = vel_z # 원본 코드에서 yaw 대신 vel_z 사용
    #     self.pub_pose.publish(pose)
        
def quaternion_from_euler(roll, pitch, yaw):
    """
    오일러 각도(롤, 피치, 요)를 쿼터니언으로 변환합니다.
    ROS 2 오도메트리 메시지의 자세 정보를 설정하는 데 사용됩니다.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr # x
    q[1] = sy * cp * sr + cy * sp * cr # y
    q[2] = sy * cp * cr - cy * sp * sr # z
    q[3] = cy * cp * cr + sy * sp * sr # w

    return q
            
def main(args=None):
    rclpy.init(args=args)

    bringup_node = BringUp()

    try:
        rclpy.spin(bringup_node)
    except KeyboardInterrupt:
        bringup_node.get_logger().info('Keyboard Interrupt: Shutting down robot node.')
    finally:
        # 노드 종료 시 모터 정지 명령 전송 및 시리얼 포트 닫기
        # 이전에 _send_stop_commands()를 호출하여 모터를 멈춥니다.
        bringup_node._send_stop_commands() 
        if bringup_node.pico_serial.is_open:
            bringup_node.pico_serial.close()
            bringup_node.get_logger().info("Serial port closed.")
        bringup_node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()


