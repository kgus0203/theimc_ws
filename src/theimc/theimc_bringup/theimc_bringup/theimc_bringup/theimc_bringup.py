import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
import math
import serial
import time
import sys
import numpy as np


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a)) 

# --- 데이터 구조 클래스 정의 ---
class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    
class Encoder:
    def __init__(self):
        self.current_data = 0.0

class CmdVel:
    def __init__(self):
        self.lin_vel_x = 0.0
        self.ang_vel_z = 0.0

class Joint:
    def __init__(self):
        self.joint_name = ['wheel_left_joint', 'wheel_right_joint']
        self.joint_pos = [0.0, 0.0]
        self.joint_vel = [0.0, 0.0]

class BringUp(Node):

    def __init__(self):
        super().__init__('bring_up')
        self.serial_port = "/dev/arduino_link"
        self.baud_rate = 115200
        self.serial_timeout = 0.1

        try:
            self.pico_serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout
            )
            self.pico_serial.reset_input_buffer()
            self.pico_serial.reset_output_buffer()
            self.get_logger().info(f"Successfully connected to serial port: {self.serial_port} at {self.baud_rate} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.serial_port}: {e}")
            rclpy.shutdown()
            sys.exit(1)
            
        
            
        # IMU 시리얼 포트 설정 및 관련 코드 삭제
        
        # --- 로봇 물리적 파라미터 ---
        self.wheel_separation = 0.780 # [m] 좌우 바퀴 간의 거리 (780mm)
        self.wheel_radius = 0.065     # [m] 바퀴의 반지름 (65mm)
        self.enc_resolution = 3600    # 엔코더 1회전당 펄스 수
        self.pulse_per_meter = self.enc_resolution / (2 * math.pi * self.wheel_radius)
        self.serial_write_delay = 0.007

        # --- 속도 제한 ---
        self.max_lin_vel_x = 0.3
        self.max_ang_vel_z = 0.5
        
        self.timestamp_previous = self.get_clock().now()
        self.pub_manual_tf = TransformBroadcaster(self)  # 수동 TF 발행용
        # 인스턴스 초기화
        self.enc_r = Encoder()
        self.enc_l = Encoder()
        self.odom_pose = OdomPose()
        self.joint = Joint()
        self.cmd_vel = CmdVel()
        
        # 오도메트리 융합을 위한 파라미터는 삭제

        # QoS 프로파일 설정
        qos_profile = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST
        )

        # ROS 2 Subscriber 설정
        self.sub_cmd_vel_msg = self.create_subscription(Twist, 'cmd_vel', self.cb_cmd_vel_msg, qos_profile)
        self.get_logger().info("Subscribing to /cmd_vel topic.")

        # ROS 2 Publisher 설정
        self.pub_joint_states = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.pub_odom = self.create_publisher(Odometry, 'odom', qos_profile)
        self.pub_odom_tf = TransformBroadcaster(self)
        self.pub_Pose = self.create_publisher(PoseStamped, 'pose', qos_profile)
        # IMU publisher 삭제
        self.get_logger().info("Publishers for /joint_states, /odom, and /pose created.")
        
        
        # 타이머 설정
        self.timer_period = 0.1 # [s] 업데이트 주기
        self.timer_proc = self.create_timer(self.timer_period, self.update_robot)
        
        self.last_sent_cmd_vel_x = 0.0
        self.last_sent_cmd_vel_z = 0.0
               

                                  
    def cb_cmd_vel_msg(self, cmd_vel_msg):
        current_lin_vel_x = cmd_vel_msg.linear.x # [m/s]
        current_ang_vel_z = cmd_vel_msg.angular.z # [rad/s]
        
        linear_deadzone = 0.01 # [m/s]
        angular_deadzone = 0.03 # [rad/s]

        # 데드존 적용
        if abs(current_lin_vel_x) < linear_deadzone:
            current_lin_vel_x = 0.0
        if abs(current_ang_vel_z) < angular_deadzone:
            current_ang_vel_z = 0.0

        # 최대 속도 제한
        current_lin_vel_x = max(-self.max_lin_vel_x, min(self.max_lin_vel_x, current_lin_vel_x))
        current_ang_vel_z = max(-self.max_ang_vel_z, min(self.max_ang_vel_z, current_ang_vel_z))

        if (abs(current_lin_vel_x - self.last_sent_cmd_vel_x) > 0.001 or
            abs(current_ang_vel_z - self.last_sent_cmd_vel_z) > 0.001):
            
            try:
                command_string = f"ROS_CMD,{current_lin_vel_x:.3f},{current_ang_vel_z:.3f}\n"
                self.pico_serial.write(command_string.encode("utf-8"))
                time.sleep(self.serial_write_delay)
                
                self.last_sent_cmd_vel_x = current_lin_vel_x
                self.last_sent_cmd_vel_z = current_ang_vel_z
            except serial.SerialException as e:
                self.get_logger().error(f"Serial communication error during motor command: {e}")
                self._send_stop_commands()

    def _send_stop_commands(self):
        try:
            self.pico_serial.write(b"ROS_CMD,0.000,0.000\n")
            time.sleep(self.serial_write_delay)
            self.last_sent_cmd_vel_x = 0.0
            self.last_sent_cmd_vel_z = 0.0
            self.get_logger().info("Sent motor stop command 'ROS_CMD,0.000,0.000'.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to send stop commands: {e}")
    
    def update_robot(self):
        self.timestamp_now = self.get_clock().now()
        dt = (self.timestamp_now - self.timestamp_previous).nanoseconds * 1e-9
        
        if dt <= 0:
            return
        if dt > 3.0:
            self.get_logger().warn(f"Abnormal dt detected ({dt:.3f}s). Odometry may be inaccurate.")
            return
            
        self.timestamp_previous = self.timestamp_now
        
        # --- 엔코더 데이터 읽기 및 파싱 ---
        trans_vel_encoder = 0.0 
        orient_vel_encoder = 0.0
        left_enc_count = 0.0
        right_enc_count = 0.0

        if self.pico_serial.is_open and self.pico_serial.in_waiting:
            latest_encoder_data = None
            # 버퍼에 남아있는 모든 라인을 읽고 최신 엔코더 데이터만 사용
            while self.pico_serial.in_waiting > 0:
                try:
                    line = self.pico_serial.readline().decode('utf-8').strip()
                    if line.startswith("ENC,"):
                        latest_encoder_data = line
                except UnicodeDecodeError:
                    self.get_logger().warn("UnicodeDecodeError when reading Pico serial. Skipping malformed line.")
                    continue
                except serial.SerialException as e:
                    self.get_logger().error(f"Serial read error on Pico: {e}")
                    break

            if latest_encoder_data:
                try:
                    parts = latest_encoder_data.split(',')
                    if len(parts) == 5 and parts[0] == "ENC":
                        left_enc_count = float(parts[1])
                        right_enc_count = float(parts[2])
                        left_enc_vel_pps = float(parts[3])
                        right_enc_vel_pps = float(parts[4])

                        self.enc_l.current_data = left_enc_count
                        self.enc_r.current_data = right_enc_count

                        lin_vel_left_mps = left_enc_vel_pps / self.pulse_per_meter
                        lin_vel_right_mps = right_enc_vel_pps / self.pulse_per_meter

                        trans_vel_encoder = (lin_vel_left_mps + lin_vel_right_mps) / 2.0
                        orient_vel_encoder = (-lin_vel_right_mps + lin_vel_left_mps) / self.wheel_separation
                    else:
                        self.get_logger().warn(f"Received malformed encoder data: {latest_encoder_data}")
                except (ValueError, IndexError) as e:
                    self.get_logger().error(f"Error parsing encoder data '{latest_encoder_data}': {e}")
            else:
                self.get_logger().debug("No new valid encoder data received from Pico this cycle.")

        # --- 오도메트리 및 조인트 상태 업데이트 ---
        self.update_odometry(trans_vel_encoder, orient_vel_encoder, dt)
        
        self.update_JointStates(left_enc_count, right_enc_count, trans_vel_encoder, orient_vel_encoder)
        self.get_logger().info(
            f"Linear vel: {trans_vel_encoder:.3f} m/s, Angular vel: {orient_vel_encoder:.3f} rad/s (Encoder)"
        )
        self.get_logger().info(
            f"Angular vel (IMU): {self.imu_latest_angular_velocity_z:.3f} rad/s"
        )
        
        
        
    def update_odometry(self, trans_vel_encoder, orient_vel_encoder, dt):

       
        self.odom_pose.theta += orient_vel_encoder * dt

        self.odom_pose.x += trans_vel_encoder * math.cos( self.odom_pose.theta) * dt
        self.odom_pose.y += trans_vel_encoder * math.sin( self.odom_pose.theta) * dt

        
        # 오일러 각도(yaw)를 쿼터니언으로 변환
        q = quaternion_from_euler(0, 0, self.odom_pose.theta)
            
        # Odometry 메시지 발행
        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.header.stamp = self.timestamp_now.to_msg()
            
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Twist (속도) 정보: 선속도와 각속도 모두 엔코더 기반
        odom.twist.twist.linear.x = trans_vel_encoder
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = orient_vel_encoder

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

    
    def update_JointStates(self, odo_l_count, odo_r_count, trans_vel_encoder, orient_vel_encoder):

        # 엔코더 펄스 수를 바퀴의 각도 (라디안)로 변환
        wheel_ang_left = (odo_l_count / self.enc_resolution) * (2 * math.pi)
        wheel_ang_right = (odo_r_count / self.enc_resolution) * (2 * math.pi)

        # 바퀴의 각속도 (rad/s) 계산 - 엔코더 기반
        wheel_ang_vel_left = (trans_vel_encoder - (self.wheel_separation / 2.0) * orient_vel_encoder) / self.wheel_radius
        wheel_ang_vel_right = (trans_vel_encoder + (self.wheel_separation / 2.0) * orient_vel_encoder) / self.wheel_radius

        self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
        self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]
            
        # JointState 메시지 발행
        joint_states = JointState()
        joint_states.header.frame_id = "base_link"
        joint_states.header.stamp = self.timestamp_now.to_msg()
        joint_states.name = self.joint.joint_name
        joint_states.position = self.joint.joint_pos
        joint_states.velocity = self.joint.joint_vel
        joint_states.effort = []
        self.pub_joint_states.publish(joint_states)
            
def quaternion_from_euler(roll, pitch, yaw):
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
        bringup_node._send_stop_commands()
        if bringup_node.pico_serial.is_open:
            bringup_node.pico_serial.close()
            bringup_node.get_logger().info("Pico serial port closed.")
        bringup_node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
