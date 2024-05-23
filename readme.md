```
noeticインストール
pyenvで3.8.10をインストール
pyenv vietualenv 3.8.10 defom
pyenv local defom; pyenv global defom
pip install empy==3.3,4 catkin_pkg pyyaml rospkg
sudo apt-get install ros-noetic-soem ros-noetic-socketcan-interface

mkdir -p DeformContact_ws/src
cd DeformContact_ws/src
git clone git@github.com:MY-CODE-1981/robotiq.git
git clone git@github.com:MY-CODE-1981/doosan-robot.git
git clone git@github.com:MY-CODE-1981/DeformContact.git
```