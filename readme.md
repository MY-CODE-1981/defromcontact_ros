```
・環境構築
noeticインストール
pyenvで3.8.10をインストール
pyenv vietualenv 3.8.10 defom
pyenv local defom; pyenv global defom
pip install empy==3.3,4 catkin_pkg pyyaml rospkg defusedxml PyQt5 PySide2
sudo apt-get install ros-noetic-soem ros-noetic-socketcan-interface

mkdir -p ~/workspace/DeformContact_ws/src
cd ~/workspace/DeformContact_ws/src
git clone git@github.com:MY-CODE-1981/defromcontact_ros.git
cd ..
catkin_make -j100  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE='/home/initial/.pyenv/versions/3.8.10/envs/defom/bin/python'
source devel/setup.bash

・実行
グリッパ表示テスト　
roslaunch robotiq_2f_85_gripper_visualization test_2f_85_model.launch

アーム表示テスト
roslaunch dsr_description m0609.launch

アームとグリッパが一体のモデルはdsr_descriptionにある。gripper変数にrobotiq_2fを設定して以下を実行
roslaunch dsr_description m0609.launch

omniverse isaac sim 2022.2.1
open_in_terminalで開くウィンドウで以下を実行
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.ros_bridge/noetic/lib;
source /home/initial/workspace/DeformContact_ws/devel/setup.bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/share/ov/pkg/isaac_sim-2023.1.1/exts/omni.isaac.ros_bridge/noetic/lib;
source /home/initial/workspace/DeformContact_ws/devel/setup.bash

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/share/ov/pkg/isaac-sim-4.0.0/exts/omni.isaac.ros_bridge/noetic/lib;
source /home/initial/workspace/DeformContact_ws/devel/setup.bash

./isaac-sim.sh

/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint.usda
->/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint_camera_v2_1.usda

剛体の場合
/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint_camera_v2_rigid_cube.usda

柔軟物体の場合
/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint_camera_v2_soft_cube.usda

omniverse://localhost/NVIDIA/Assets/Isaac/2022.2.1/Isaac/Samples/ROS2/Scenario/carter_warehouse_apriltags_worker.usd

moveit
☓roslaunch moveit_config_m0609_robotiq_2f_85 demo.launch

roslaunch sim_isaac isaac_execution.launch

☓rosrun sim_isaac convert_32SC1_to_8UC1.py

☓rosrun sim_isaac publish_camera_info.py

rosrun sim_isaac pose_planner_pose.py

roslaunch learning_stereo depth_estimation_doosan.launch

・その他
モデルファイル作成
rosrun xacro xacro --inorder [absolute path]/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/xacro/m0609_robotiq_2f_85.urdf.xacro > [absolute path]//DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/urdf/m0609_robotiq_2f_85.urdf

Ex) rosrun xacro xacro --inorder $HOME/workspace/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/xacro/m0609_robotiq_2f_85.urdf.xacro > $HOME/workspace/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/urdf/m0609_robotiq_2f_85.urdf

Ex) rosrun xacro xacro --inorder $HOME/workspace/DeformContact_ws/src/defromcontact_ros/realsense/realsense-ros/realsense2_description/urdf/test_d435i_camera.urdf.xacro > $HOME/workspace/DeformContact_ws/src/defromcontact_ros/realsense/realsense-ros/realsense2_description/urdf/test_d435i_camera.urdf


moveit_config_m0609にアームのmoveitがあるがgripper部分がないため以下で作成しなおす
roslaunch moveit_setup_assistant setup_assistant.launch

isaac用のパッケージを作成
catkin_create_pkg sim_isaac std_msgs rospy roscpp

isaac_sim 2022.2.1でロボットモデルを作成

open_in_terminalで開くウィンドウで以下を実行
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.ros_bridge/noetic/lib;
source /home/initial/workspace/DeformContact_ws/devel/setup.bash
./isaac-sim.sh
/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint.usdaを開く
/home/initial/workspace/homer_ws/src/homer/models/isaac/test_composer/test.usdaはcloth

上記に以下をインポート
/home/initial/workspace/recognition_e2dr_project_shared/src/sim_based_recog/object_description/urdf/cloth/
cc
/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/m0609_robotiq_2f_85

urdf内のパッケージ名指定の相対リンクを認識してstlファイルを読み込める様になる

action_graphで以下の様なエラーがでたらtarge_primの設定をロボット名でなくルートリンクにかえる
"prime s not an articulation ros1 publish joint state"
"pattern did not match any articulations"
https://forums.developer.nvidia.com/t/pattern-world-fancy-robot-did-not-match-any-articulations/283975/2

2024/05/25の続き
/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/doosan-robot/moveit_config_m0609_robotiq_2f_85/config
のファイルをsim_envを参考にisaacとmoveitがつながる様に調整する。おそらく、微妙な違いがある
```