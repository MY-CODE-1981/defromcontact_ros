#!/home/initial/.pyenv/versions/3.8.10/envs/defom/bin/python

import rospy
from sensor_msgs.msg import CameraInfo
import numpy as np

def publish_camera_info():
    # ROSノードを初期化
    rospy.init_node('camera_info_publisher', anonymous=True)
    # パブリッシャーを作成
    pub = rospy.Publisher('/camera/infra1/camera_info_scaled', CameraInfo, queue_size=10)
    
    # ループのレートを設定
    rate = rospy.Rate(10) # 10Hz
    
    # CameraInfoメッセージを作成
    camera_info_msg = CameraInfo()
    camera_info_msg.header.frame_id = "camera_link"
    camera_info_msg.width = 640
    camera_info_msg.height = 480
    
    # カメラ行列の設定
    K = [381.8360595703125, 0.0, 320.25006103515625,
         0.0, 381.8360595703125, 239.46475219726562,
         0.0, 0.0, 1.0]
    
    camera_info_msg.K = K
    
    # ディストーションパラメータの設定
    camera_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
    camera_info_msg.distortion_model = "plumb_bob"
    
    # リプロジェクション行列の設定
    camera_info_msg.P = [K[0], 0.0, K[2], 0.0, 
                         0.0, K[4], K[5], 0.0, 
                         0.0, 0.0, 1.0, 0.0]
    
    # 回転行列の設定
    camera_info_msg.R = [1.0, 0.0, 0.0, 
                         0.0, 1.0, 0.0, 
                         0.0, 0.0, 1.0]
    
    while not rospy.is_shutdown():
        # タイムスタンプを更新
        camera_info_msg.header.stamp = rospy.Time.now()
        # メッセージをパブリッシュ
        pub.publish(camera_info_msg)
        # ループレートに従ってスリープ
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_camera_info()
    except rospy.ROSInterruptException:
        pass
