#!/home/initial/.pyenv/versions/3.8.10/envs/defom/bin/python
import geometry_msgs.msg
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
import moveit_commander
import numpy as np
import roslib.packages
import rospy
from rotations import rotations as rt
import sys
import tf
import tf2_ros
import time
# from xarm_control.srv import image_save
import multiprocessing
import os

class PosePlanner(object):
    def __init__(self, filepath):
        data = np.load(filepath)
        self.pos_list = data['translate']
        self.pos_list[2] = self.pos_list[2] + 0.07
        self.ori_list = data['matrix']
        self.quat = data['quat']
        self.rpy = data['rpy']
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.count = 0
        # self.init_pose()
        moveit_commander.roscpp_initialize(sys.argv)
        self.arm_move_group = moveit_commander.MoveGroupCommander("arm")
        self.arm_move_group.set_planning_time(10.0)
        self.arm_move_group.set_goal_position_tolerance(0.005)
        self.arm_move_group.set_goal_orientation_tolerance(0.005)
        self.arm_move_group.set_num_planning_attempts(10) 
        self.gripper_move_group = moveit_commander.MoveGroupCommander("gripper")
        self.gripper_move_group.set_planning_time(10.0)
        self.gripper_move_group.set_goal_position_tolerance(0.005)
        self.gripper_move_group.set_goal_orientation_tolerance(0.005)
        self.gripper_move_group.set_num_planning_attempts(10) 
        # joint_init = [0.4833, 0.0371, 1.8744, 0.0000, 1.2269, 3.0115]
        joint_init = np.deg2rad([38, -3, 116, 0, 68, 183])
        self.init_pose(joint_init)
        
    def pose_to_transform(self, pose):
        transform = Transform()
        transform.translation.x = pose[0]
        transform.translation.y = pose[1]
        transform.translation.z = pose[2]
        transform.rotation.x = pose[3]
        transform.rotation.y = pose[4]
        transform.rotation.z = pose[5]
        transform.rotation.w = pose[6]
        return transform

    def init_pose(self, joint_goal):        
        # tfを発行
        # initial_pose = TransformStamped()
        # initial_pose.header.stamp = rospy.Time.now()
        # initial_pose.header.frame_id = 'base_0'
        # initial_pose.child_frame_id = 'tf_goal'
        # initial_pose.transform = self.pose_to_transform(joint_goal)
        # self.broadcaster.sendTransform(initial_pose)
        try:
            # moveit_commander.roscpp_initialize(sys.argv)

            # ノードの生成
            # MoveGroupCommanderの準備
            # move_group = moveit_commander.MoveGroupCommander("arm")

            # 関節の角度でゴール状態を指定
            self.arm_move_group.set_joint_value_target(joint_goal)
            # self.arm_move_group.set_num_planning_attempts(10) 
            # モーションプランの計画と実行
            success = self.arm_move_group.go(wait=True)
            
            # 結果の確認
            if success:
                rospy.loginfo("Motion plan was successful")
            else:
                rospy.logwarn("Motion plan failed")
            
            # 後処理
            self.arm_move_group.clear_pose_targets()
            self.arm_move_group.stop()
        
        except BaseException:
            pass
        

    # def image_save_control_client(self, save_request):
    #     rospy.wait_for_service('image_save_control')
    #     try:
    #         image_save_control = rospy.ServiceProxy('image_save_control', image_save)
    #         resp1 = image_save_control(save_request)
    #         return resp1.save_answer
    #     except BaseException:
    #         pass

    def calc(self):
        # joint_init = [0.4833, 0.0371, 1.8744, 0.0000, 1.2269, 3.0115]
        # self.init_pose(joint_init)
        # save_answer = self.image_save_control_client(False)
        # i = self.count
        # (trans_x, trans_y, trans_z) = self.pos_list[i]
        # (ori_r, ori_p, ori_y) = rt.rotmat2rpy(self.ori_list[i])
        # (quat_x, quat_y, quat_z, quat_w) = rt.rotmat2quat(self.ori_list[i])

        # (quat_x, quat_y, quat_z, quat_w) = rt.rotmat2quat(self.ori_list)
        # pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.position = Vector3(trans_x, trans_y, trans_z)
        # q = tf.transformations.quaternion_from_euler(ori_r, ori_p, ori_y)
        # pose_goal.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        
        # tfを発行
        # command_pose = TransformStamped()
        # command_pose.header.stamp = rospy.Time.now()
        # command_pose.header.frame_id = 'link_base'
        # command_pose.child_frame_id = 'tf_goal'
        # command_pose.transform = self.pose_to_transform(joint_goal)
        # self.broadcaster.sendTransform(command_pose)
        
        ###
        # print(trans_x, trans_y, trans_z)
        # print(ori_r, ori_p, ori_y)

        try:
            # MoveitCommanderの初期化
            moveit_commander.roscpp_initialize(sys.argv)
            self.arm_move_group = moveit_commander.MoveGroupCommander("arm")
            self.arm_move_group.set_planning_time(10.0)
            self.arm_move_group.set_goal_position_tolerance(0.005)
            self.arm_move_group.set_goal_orientation_tolerance(0.005)
            self.arm_move_group.set_num_planning_attempts(10) 
            self.gripper_move_group = moveit_commander.MoveGroupCommander("gripper")
            self.gripper_move_group.set_planning_time(10.0)
            self.gripper_move_group.set_goal_position_tolerance(0.005)
            self.gripper_move_group.set_goal_orientation_tolerance(0.005)
            self.gripper_move_group.set_num_planning_attempts(10) 

            # ノードの生成
            # MoveGroupCommanderの準備
            # move_group = moveit_commander.MoveGroupCommander("arm")
            # move_group.set_planning_time(10.0)
            # move_group.set_goal_position_tolerance(0.005)
            # move_group.set_goal_orientation_tolerance(0.005)

            # エンドポイントの姿勢でゴール状態を指定
            joint_init = np.deg2rad([38, 3, 120, 0, 57, 183])
            # joint_init = np.deg2rad([38, -3, 116, 0, 68, 183])
            self.init_pose(joint_init)
            # self.arm_move_group.set_goal_position_tolerance(0.001)
            # self.arm_move_group.set_goal_orientation_tolerance(0.05)
            # self.arm_move_group.set_planner_id("RRTConnectkConfigDefault")

            # # エンドポイントの姿勢でゴール状態を指定
            # pose_goal = geometry_msgs.msg.Pose()
            # (trans_x, trans_y, trans_z) = self.pos_list
            # (ori_r, ori_p, ori_y) = rt.rotmat2rpy(self.ori_list)
            # pose_goal.position = Vector3(trans_x, trans_y, trans_z)
            # q = tf.transformations.quaternion_from_euler(ori_r, ori_p, ori_y)
            # pose_goal.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            # self.arm_move_group.set_pose_target(pose_goal)

            # # # モーションプランの計画と実行
            # success = self.arm_move_group.go(wait=True)
            
            # if success:
            #     rospy.loginfo("Motion plan was successful")
            # else:
            #     rospy.logwarn("Motion plan failed")
                
            # # 後処理
            self.arm_move_group.stop()
            self.arm_move_group.clear_pose_targets()
            
        except BaseException:
            pass

        self.count += 1

        # save_answer = self.image_save_control_client(True)

        time.sleep(2)

        # if self.count == len(self.pos_list) / 2:
        #     self.init_pose()

        # if save_answer is True:
        #     save_answer = self.image_save_control_client(False)
        # else:
        #     print("miss to save")

    def spin_once(self):
        rospy.init_node('spin_once_' + str(os.getpid()), anonymous=True)
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                self.calc()
                r.sleep()
            except rospy.exceptions.ROSInterruptException:
                break

    def pub_cameras_tf(self, target_frame, source_frame, trans, quat):
        rospy.init_node('pub_cameras_tf_' + str(os.getpid()), anonymous=True)
        rate = rospy.Rate(10.0)
        br = tf2_ros.TransformBroadcaster()
        while not rospy.is_shutdown():
            # print(target_frame, source_frame)
            # to publish camera_tf
            t = geometry_msgs.msg.TransformStamped()
    
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = target_frame
            t.child_frame_id = source_frame
            
            t.transform.translation.x = trans[0]
            t.transform.translation.y = trans[1]
            t.transform.translation.z = trans[2]
            t.transform.rotation.x = quat[0]
            t.transform.rotation.y = quat[1]
            t.transform.rotation.z = quat[2]
            t.transform.rotation.w = quat[3]
            br.sendTransform(t)
            rate.sleep()
    
if __name__ == '__main__':
    # rospy.init_node('calibration_transform_obj')
    pkg_name = 'sim_isaac'
    path_root = roslib.packages.get_pkg_dir(pkg_name)
    path = path_root + '/output/pose_cube/pose.npz'
    mock = PosePlanner(path)
    process1 = multiprocessing.Process(target=mock.pub_cameras_tf, args=("world", "cube_mesh", mock.pos_list, mock.quat))
    # process2 = multiprocessing.Process(target=mock.spin_once)
    process1.start()
    # process2.start()
    while True:
        mock.spin_once()
        time.sleep(0.1)

    process1.terminate()
    # process2.terminate()
    process1.join()
    # process2.join()
