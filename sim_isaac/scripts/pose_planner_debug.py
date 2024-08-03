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


class PosePlanner(object):
    def __init__(self, filepath):
        data = np.load(filepath)
        self.pos_list = data['translate']
        self.ori_list = data['matrix']
        self.broadcaster = tf2_ros.TransformBroadcaster()
        self.count = 0
        # self.init_pose()
        joint_init = [0.4833, 0.0371, 1.8744, 0.0000, 1.2269, 3.0115]
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
            move_group = moveit_commander.MoveGroupCommander("arm")

            # 関節の角度でゴール状態を指定
            move_group.set_joint_value_target(joint_goal)
            move_group.set_num_planning_attempts(10) 
            # モーションプランの計画と実行
            success = move_group.go(wait=True)
            # 後処理
            move_group.stop()

        except BaseException:
            pass
            
        # 結果の確認
        if success:
            rospy.loginfo("Motion plan was successful")
        else:
            rospy.logwarn("Motion plan failed")
        
        # 後処理
        move_group.stop()
        move_group.clear_pose_targets()

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
        i = self.count
        # (trans_x, trans_y, trans_z) = self.pos_list[i]
        # (ori_r, ori_p, ori_y) = rt.rotmat2rpy(self.ori_list[i])
        # (quat_x, quat_y, quat_z, quat_w) = rt.rotmat2quat(self.ori_list[i])
        (trans_x, trans_y, trans_z) = self.pos_list
        (ori_r, ori_p, ori_y) = rt.rotmat2rpy(self.ori_list)
        (quat_x, quat_y, quat_z, quat_w) = rt.rotmat2quat(self.ori_list)
        
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

            # ノードの生成
            # MoveGroupCommanderの準備
            move_group = moveit_commander.MoveGroupCommander("arm")
            move_group.set_planning_time(10.0)
            move_group.set_goal_position_tolerance(0.005)
            move_group.set_goal_orientation_tolerance(0.005)

            # エンドポイントの姿勢でゴール状態を指定
            pose_goal = geometry_msgs.msg.Pose()
            pose_goal.position = Vector3(trans_x, trans_y, trans_z)
            q = tf.transformations.quaternion_from_euler(ori_r, ori_p, ori_y)
            pose_goal.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
            move_group.set_pose_target(pose_goal)

            # # モーションプランの計画と実行
            success = move_group.go(wait=True)
            
            if success:
                rospy.loginfo("Motion plan was successful")
            else:
                rospy.logwarn("Motion plan failed")
                
            # # 後処理
            move_group.stop()
            move_group.clear_pose_targets()
            
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
        self.calc()


if __name__ == '__main__':
    rospy.init_node('calibration_transform_obj')
    r = rospy.Rate(10.0)
    pkg_name = 'sim_isaac'
    path_root = roslib.packages.get_pkg_dir(pkg_name)
    path = path_root + '/output/pose_cube/pose.npz'
    mock = PosePlanner(path)
    while not rospy.is_shutdown():
        try:
            mock.spin_once()
            r.sleep()
        except rospy.exceptions.ROSInterruptException:
            break
