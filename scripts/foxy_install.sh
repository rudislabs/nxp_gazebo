
#Install ROS2 Foxy
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install -y  gnupg2 \
	lsb-release \
	ros-foxy-ros-base \
	ros-foxy-desktop \
	ros-foxy-cv-bridge \
	ros-foxy-image-tools \
	ros-foxy-image-transport \
	ros-foxy-image-transport-plugins \
	ros-foxy-image-pipeline \
	ros-foxy-camera-calibration-parsers \
	ros-foxy-camera-info-manager \
	ros-foxy-launch-testing-ament-cmake \
	ros-foxy-cv-bridge \
	ros-foxy-vision-opencv \
	ros-foxy-vision-msgs \
	ros-foxy-vision-msgs-dbgsym

source /opt/ros/foxy/setup.bash
sudo apt install -y python3-pip
pip3 install -U argcomplete
sudo apt-get install python3-colcon-common-extensions
echo 'source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash' >> ~/.bashrc
