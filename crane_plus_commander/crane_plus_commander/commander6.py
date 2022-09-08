import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from threading import Event
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from airobot_interfaces.srv import StringCommand
import time
import threading
from math import pi
from crane_plus_commander.kbhit import KBHit
from crane_plus_commander.kinematics import (forward_kinematics, from_gripper_ratio, gripper_in_range,
    inverse_kinematics, joint_in_range, to_gripper_ratio)
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


# CRANE+ V2用のアクションへリクエストを送り，他からサービスを受け付けるノード
class Commander(Node):

    def __init__(self, timer=False):
        super().__init__('commander')
        self.callback_group = ReentrantCallbackGroup()
        self.joint_names = [
            'crane_plus_joint1',
            'crane_plus_joint2',
            'crane_plus_joint3',
            'crane_plus_joint4']
        self.gripper_names = [
            'crane_plus_joint_hand']
        self.action_client_joint = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_arm_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.action_client_gripper = ActionClient(
            self, FollowJointTrajectory,
            'crane_plus_gripper_controller/follow_joint_trajectory',
            callback_group=self.callback_group)
        self.action_done_event = Event()
        self.elbow_up = True
        # 文字列とポーズの組を保持する辞書
        self.poses = {}
        self.poses['zeros'] = [0, 0, 0, 0]
        self.poses['ones'] = [1, 1, 1, 1]
        self.poses['home'] = [0.0, -1.16, -2.01, -0.73]
        self.poses['carry'] = [-0.00, -1.37, -2.52, 1.17]
        self.poses['open'] = [0, 0, 0, -0.69]
        self._tf_publisher = StaticTransformBroadcaster(self)
        self.send_static_transform()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self.service = self.create_service(
            StringCommand, 'manipulation/command', self.command_callback,
            callback_group=self.callback_group)
        
        
        
        #self.endtip = {}

    def command_callback(self, request, response):
        self.get_logger().info(f'command: {request.command}')
        words = request.command.split()
        if words[0] == 'set_pose':
            self.set_pose(words, response)
        elif words[0] == 'set_gripper':
            self.set_gripper(words, response)
        elif words[0] == 'set_endtip':
            self.set_endtip(words, response)
        elif words[0] == 'pickup':
            self.set_pickup(words, response)
        else:
            response.answer = f'NG {words[0]} not supported'
        self.get_logger().info(f'answer: {response.answer}')
        return response
        
        
    def set_pickup(self, words, response):
        if len(words) < 2:
            response.answer = f'NG {words[0]} argument required'
            return
        print('a')
        position = self.get_endtip_position(words[1])
        if position is not None:
            x, y, z, roll, pitch, yaw = position
            print((f'x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, '
                   f'roll: {roll:.3f}, pitch: {pitch:.3f}, '
                   f'yaw: {yaw:.3f}'))
          #逆運動学入れる
        words[0]={}
        words[0][str(words[1])] = [x,y,z,pitch]
        words[0][str(words[1])][0]= words[0][str(words[1])][0] - 0.12
        print(str(words[0][str(words[1])]))
        self.joint = inverse_kinematics(words[0][str(words[1])], self.elbow_up)
        print(str(self.joint))
        if self.joint is None:
                        print('逆運動学の解なし')
                        response.answer = '逆運動学の解なし'
                        return
        r = self.send_goal_joint(self.joint, 3.0)
        if self.check_action_result(r, response):
            return        
        response.answer = '3sec stop'     
        #next gripper => open 
        gripper = -0.70
        dt = 1.0
        r = self.send_goal_gripper(gripper, dt)
        if self.check_action_result(r, response):
            return
        time.sleep(3)
        response.answer = '3sec stop'  
        
        #next arm => move 
        words[0][str(words[1])][0]=words[0][str(words[1])][0] + 0.1
        
        self.joint = inverse_kinematics(words[0][str(words[1])], self.elbow_up)
        print(str(self.joint))
        if self.joint is None:
            print('逆運動学の解なし')
            response.answer = '逆運動学の解なし'
            return
        r = self.send_goal_joint(self.joint, 3.0)
        if self.check_action_result(r, response):
            return 
        time.sleep(3)
        response.answer ='3sec stop'
        
        
        #next gripper => close
        gripper = 0
        dt = 1.0
        r = self.send_goal_gripper(gripper, dt)
        if self.check_action_result(r, response):
            return
        
        #next arm => move 
        words[0][str(words[1])][2]=words[0][str(words[1])][2] + 	0.04
        
        self.joint = inverse_kinematics(words[0][str(words[1])], self.elbow_up)
        print(str(self.joint))
        if self.joint is None:
            print('逆運動学の解なし')
            response.answer = '逆運動学の解なし'
            return
        r = self.send_goal_joint(self.joint, 3.0)
        if self.check_action_result(r, response):
            return
        time.sleep(3)
        response.answer='3sec stop'
        
        
        words[0][str(words[1])][0]=words[0][str(words[1])][0] - 0.1
        
        self.joint = inverse_kinematics(words[0][str(words[1])], self.elbow_up)
        print(str(self.joint))
        if self.joint is None:
            print('逆運動学の解なし')
            response.answer = '逆運動学の解なし'
            return
        r = self.send_goal_joint(self.joint, 3.0)
        if self.check_action_result(r, response):
            return 
        time.sleep(3)
        response.answer = 'OK'
        
        
        
    def set_pose(self, words, response):
        if len(words) < 2:
            response.answer = f'NG {words[0]} argument required'
            return
        if not words[1] in self.poses:
            response.answer = f'NG {words[1]} not found'
            return
        r = self.send_goal_joint(self.poses[words[1]], 3.0)
        if self.check_action_result(r, response):
            return
        response.answer = 'OK'
        
        
    def set_endtip(self, words, response):
        if len(words) < 2:
            response.answer = f'NG {words[0]} argument required'
            return
        print('a')
        position = self.get_endtip_position()
        if position is not None:
            x, y, z, roll, pitch, yaw = position
            print((f'x: {x:.3f}, y: {y:.3f}, z: {z:.3f}, '
                   f'roll: {roll:.3f}, pitch: {pitch:.3f}, '
                   f'yaw: {yaw:.3f}'))
          #逆運動学入れる
        self.joint = inverse_kinematics([x, y, z, pitch], self.elbow_up)
        print(str(self.joint))
        if self.joint is None:
                        print('逆運動学の解なし')
                        response.answer = '逆運動学の解なし'
                        return
        r = self.send_goal_joint(self.joint, 3.0)
        if self.check_action_result(r, response):
            return
        response.answer = 'OK'


    def set_gripper(self, words, response):
        if len(words) < 2:
            response.answer = f'NG {words[0]} argument required'
            return
        try:
            gripper_ratio = float(words[1])
        except ValueError:
            response.answer = f'NG {words[1]} unsuitable'
            return
        gripper = from_gripper_ratio(gripper_ratio)
        if not gripper_in_range(gripper):
            response.answer = 'NG out of range'
            return
        dt = 1.0
        r = self.send_goal_gripper(gripper, dt)
        if self.check_action_result(r, response):
            return
        response.answer = 'OK'

    def check_action_result(self, r, response, message=''):
        if message != '':
            message += ' '
        if r is None:
            response.answer = f'NG {message}timeout'
            return True
        if r.result.error_code != 0:
            response.answer = f'NG {message}error_code: {r.result.error_code}'
            return True
        return False

    def send_goal_joint(self, q, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [
            float(q[0]), float(q[1]), float(q[2]), float(q[3])]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_done_event.clear()
        send_goal_future = self.action_client_joint.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.action_done_event.wait(time*2)
        return self.action_result

    def send_goal_gripper(self, gripper, time):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.header.stamp = self.get_clock().now().to_msg()
        goal_msg.trajectory.joint_names = self.gripper_names
        goal_msg.trajectory.points = [JointTrajectoryPoint()]
        goal_msg.trajectory.points[0].positions = [float(gripper)]
        goal_msg.trajectory.points[0].time_from_start = Duration(
            seconds=int(time), nanoseconds=(time-int(time))*1e9).to_msg()
        self.action_done_event.clear()
        send_goal_future = self.action_client_gripper.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.action_result = None
        self.action_done_event.wait(time*2)
        
        return self.action_result

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.action_result = future.result()
        self.action_done_event.set()
        
    def send_static_transform(self):
        st = TransformStamped()
        st.header.stamp = self.get_clock().now().to_msg()
        st.header.frame_id = 'crane_plus_link4'
        st.child_frame_id = 'crane_plus_endtip'
        st.transform.translation.x = 0.0
        st.transform.translation.y = 0.0
        st.transform.translation.z = 0.090
        qu = quaternion_from_euler(0.0, -pi/2, 0.0)
        st.transform.rotation.x = qu[0]
        st.transform.rotation.y = qu[1]
        st.transform.rotation.z = qu[2]
        st.transform.rotation.w = qu[3]
        self._tf_publisher.sendTransform(st)
    
    def get_endtip_position(self,name):
        try:
            when = rclpy.time.Time()
            trans = self._tf_buffer.lookup_transform(
                'crane_plus_base',
                name,
                when,
                timeout=Duration(seconds=1.0))
        except LookupException as e:
            self.get_logger().info(e)
            return None
        tx = trans.transform.translation.x
        ty = trans.transform.translation.y
        tz = trans.transform.translation.z
        rx = trans.transform.rotation.x
        ry = trans.transform.rotation.y
        rz = trans.transform.rotation.z
        rw = trans.transform.rotation.w
        roll, pitch, yaw = euler_from_quaternion([rx, ry, rz, rw])
        return tx, ty, tz, roll, pitch, yaw
     
    
    
    
def main():
    print('開始')

    # ROSクライアントの初期化
    rclpy.init()

    # ノードクラスのインスタンス
    commander = Commander()

    # 初期ポーズへゆっくり移動させる
    commander.send_goal_joint(commander.poses['home'], 5)
    commander.send_goal_gripper(from_gripper_ratio(1), 1)
    print('サービスサーバ待機')

    # Ctrl+cでエラーにならないようにKeyboardInterruptを捕まえる
    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(commander, executor)
        
        
    except KeyboardInterrupt:
        pass

    print('サービスサーバ停止')
    # 終了ポーズへゆっくり移動させる
    commander.send_goal_joint(commander.poses['zeros'], 5)
    commander.send_goal_gripper(from_gripper_ratio(0), 1)

    rclpy.shutdown()
    print('終了')
