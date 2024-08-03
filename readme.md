```
・環境構築
noeticインストール
pyenvで3.8.10をインストール
pyenv vietualenv 3.8.10 defom
pyenv local defom
pyenv global defom
pip install empy==3.3,4 catkin_pkg pyyaml rospkg defusedxml PyQt5 PySide2
sudo apt-get install ros-noetic-soem ros-noetic-socketcan-interface

mkdir -p ~/workspace/DeformContact_ws/src
cd ~/workspace/DeformContact_ws/src
git clone git@github.com:MY-CODE-1981/defromcontact_ros.git
cd ..
catkin_make -j100  --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE='$HOME/.pyenv/versions/3.8.10/envs/defom/bin/python'
source devel/setup.bash

・オムニバースのインストールと実行準備
omniverse isaac sim 2022.2.1
open_in_terminalで開くウィンドウで以下を実行
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/.local/share/ov/pkg/isaac_sim-2022.2.1/exts/omni.isaac.ros_bridge/noetic/lib;
source $HOME/workspace/DeformContact_ws/devel/setup.bash
./isaac-sim.sh

剛体の場合は以下を読み込み
$HOME/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint_camera_v2_rigid_cube.usda

柔軟物体の場合は以下を読み込み
$HOME/workspace/DeformContact_ws/src/defromcontact_ros/sim_isaac/model/isaac/test_env_virtual_joint_camera_v2_soft_cube.usda

・実行
１つめのターミナルで以下を実行
roscore
このあとでisaac_simのシミュレーションの開始ボタンを実行。
２つめのターミナルで以下を実行
roslaunch sim_isaac isaac_execution.launch
このあとでfoundationposeを実行。
物体検出結果がrvizに表示されたあとで３つめのターミナルで以下を事項
rosrun sim_isaac pose_planner_pose.py
```