#!/home/initial/.pyenv/versions/3.8.10/envs/defom/bin/python

import sys
import rospy
import numpy as np
from sensor_msgs.msg import Image


UPDATE_MIN, UPDATE_MAX = False, False

def republish_image_infra1(image):
    # print("callback")
    global min_value, max_value
    array = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, image.width, -1)
    min_value = min(np.min(array).item(), min_value) if UPDATE_MIN else min_value
    max_value = max(np.max(array).item(), max_value) if UPDATE_MAX else max_value
    array = np.clip((array - min_value) / (max_value - min_value + sys.float_info.min) * 255, 0, 255)
    image.data = array.astype(np.uint8).tobytes()
    image.encoding = "mono8"
    pub_infra1.publish(image)

def republish_image_infra2(image):
    # print("callback")
    global min_value, max_value
    array = np.frombuffer(image.data, dtype=np.float32).reshape(image.height, image.width, -1)
    min_value = min(np.min(array).item(), min_value) if UPDATE_MIN else min_value
    max_value = max(np.max(array).item(), max_value) if UPDATE_MAX else max_value
    array = np.clip((array - min_value) / (max_value - min_value + sys.float_info.min) * 255, 0, 255)
    image.data = array.astype(np.uint8).tobytes()
    image.encoding = "mono8"
    pub_infra2.publish(image)

if __name__ == "__main__":

    rospy.init_node("convert_32SC1_to_8UC1f")


    # get configuration from ROS parameters
    input_topic_infra1 = rospy.get_param("~input_topic", "camera/infra1/image_label")
    output_topic_infra1 = rospy.get_param("~output_topic", "camera/infra1/image_label_conv")
    input_topic_infra2 = rospy.get_param("~input_topic", "camera/infra2/image_label")
    output_topic_infra2 = rospy.get_param("~output_topic", "camera/infra2/image_label_conv")
    min_value = rospy.get_param("~min_value", None)
    max_value = rospy.get_param("~max_value", None)

    UPDATE_MIN = min_value is None
    UPDATE_MAX = max_value is None
    min_value = np.Inf if min_value is None else min_value
    max_value = -np.Inf if max_value is None else max_value

    # topics
    sub_infra1 = rospy.Subscriber(input_topic_infra1, Image, republish_image_infra1)
    pub_infra1 = rospy.Publisher(output_topic_infra1, Image, queue_size=10)
    sub_infra2 = rospy.Subscriber(input_topic_infra2, Image, republish_image_infra2)
    pub_infra2 = rospy.Publisher(output_topic_infra2, Image, queue_size=10)
    
    # spin the node
    rospy.spin()
