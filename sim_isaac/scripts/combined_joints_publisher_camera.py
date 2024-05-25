#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

joints_dict = {}


def joint_states_callback(message):

    joint_commands = JointState()

    joint_commands.header = message.header
    print("")
    for i, name in enumerate(message.name):

        # Storing arm joint names and positions
        joints_dict[name] = message.position[i]
        # print("name: ", name)
        # if name == "shoulder_pan_joint":

        #     joints_dict["shoulder_lift_joint"] = 0.0
        if name == "robotiq_85_left_knuckle_joint":
            joints_dict["robotiq_85_right_knuckle_joint"] = message.position[i]
            joints_dict["robotiq_85_left_inner_knuckle_joint"] = message.position[i]
            joints_dict["robotiq_85_right_inner_knuckle_joint"] = message.position[i]

            joints_dict["robotiq_85_left_finger_tip_joint"] = message.position[i] * -1.0
            joints_dict["robotiq_85_right_finger_tip_joint"] = message.position[i] * -1.0

    
    joint_commands.name = joints_dict.keys()
    joint_commands.position = joints_dict.values()
    print(joint_commands)

    # Publishing combined message containing all arm and finger joints
    pub.publish(joint_commands)

    return


if __name__ == "__main__":
    rospy.init_node("combined_joints_publisher")
    pub = rospy.Publisher("/joint_command", JointState, queue_size=1)
    rospy.Subscriber("/joint_command_desired", JointState,
                     joint_states_callback, queue_size=1)
    rospy.spin()


# name:  RArm_EL_joint
# name:  RArm_SHBZ_joint
# name:  RArm_SHX_joint
# name:  RArm_SHY_joint
# name:  RArm_SHZ_joint
# name:  RArm_WRX_joint
# name:  RHand_I1Y
# name:  RHand_I1Z
# name:  RHand_I2Y
# name:  RHand_I3Y
# name:  RHand_M1Y
# name:  RHand_M1Z
# name:  RHand_M2Y
# name:  RHand_M3Y
# name:  RHand_R1Y
# name:  RHand_R1Z
# name:  RHand_R2Y
# name:  RHand_R3Y
# name:  RHand_T1Y
# name:  RHand_T1Z
# name:  RHand_T2Y
# name:  RHand_T3Y
# name:  RHand_WRY
# name:  RHand_WRZ
# header: 
#   seq: 0
#   stamp: 
#     secs: 58
#     nsecs:      3025
#   frame_id: ''
# name: dict_keys(['RArm_EL_joint', 'RArm_SHBZ_joint', 'RArm_SHX_joint', 'RArm_SHY_joint', 'RArm_SHZ_joint', 'RArm_WRX_joint', 'RHand_I1Y', 'RHand_I1Z', 'RHand_I2Y', 'RHand_I3Y', 'RHand_M1Y', 'RHand_M1Z', 'RHand_M2Y', 'RHand_M3Y', 'RHand_R1Y', 'RHand_R1Z', 'RHand_R2Y', 'RHand_R3Y', 'RHand_T1Y', 'RHand_T1Z', 'RHand_T2Y', 'RHand_T3Y', 'RHand_WRY', 'RHand_WRZ'])
# position: dict_values([-1.2458, -0.2702, -0.7428, 0.6101, -0.3621, 1.1681, 0.1673, 0.1171, 0.1673, 0.1673, 0.1673, -0.0777, 0.1673, 0.1673, 0.1673, -0.0777, 0.1673, 0.1673, 0.5816, -1.5707, 0.575, 0.5798, 0.3218, -0.197])
# velocity: []
# effort: []