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
catkin_make -j100
source devel/setup.bash

・実行
グリッパ表示テスト　
roslaunch robotiq_2f_85_gripper_visualization test_2f_85_model.launch

アーム表示テスト
roslaunch dsr_description m0609.launch

アームとグリッパが一体のモデルはdsr_descriptionにある。gripper変数にrobotiq_2fを設定して以下を実行
roslaunch dsr_description m0609.launch

moveit
roslaunch moveit_config_m0609_robotiq_2f_85 demo.launch

roslaunch sim_isaac isaac_execution.launch


・その他
モデルファイル作成
rosrun xacro xacro --inorder [absolute path]/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/xacro/m0609_robotiq_2f_85.urdf.xacro > [absolute path]//DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/urdf/m0609_robotiq_2f_85.urdf
Ex) rosrun xacro xacro --inorder $HOME/workspace/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/xacro/m0609_robotiq_2f_85.urdf.xacro > $HOME/workspace/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/urdf/m0609_robotiq_2f_85.urdf

moveit_config_m0609にアームのmoveitがあるがgripper部分がないため以下で作成しなおす
roslaunch moveit_setup_assistant setup_assistant.launch

isaac用のパッケージを作成
catkin_create_pkg sim_isaac std_msgs rospy roscpp

isaac_sim 2022.2.1でロボットモデルを作成

open_in_terminalで開くウィンドウで以下を実行
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.ros_bridge/noetic/lib;
source /home/initial/workspace/DeformContact_ws/devel/setup.bash
./isaac-sim.sh
/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env.usdaを開く

/home/initial/workspace/DeformContact_ws/src/defromcontact_ros/doosan-robot/dsr_description/urdf/m0609_robotiq_2f_85.urdf
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