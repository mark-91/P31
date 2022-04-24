
INSTALLATION AND PREPARATION:


	cd ~
	sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
	sudo apt install curl
	curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
	sudo apt update
	sudo apt install ros-noetic-desktop-full
	echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc
	source ~/.zshrc
	sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
	sudo apt install python3-rosdep
	sudo rosdep init
	rosdep update
	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/
	catkin_make
	echo "~/catkin_ws/devel/setup.bash" >> ~/.zshrc
	
	cd ~
	git clone --recursive https://github.com/ArduPilot/ardupilot.git
	cd ardupilot
	Tools/environment_install/install-prereqs-ubuntu.sh -y
	. ~/.profile
	./waf configure --board CubeBlack
	./waf plane
	echo "export PATH=$PATH:$HOME/ardupilot/Tools/autotest" >> ~/.zshrc
	echo "export PATH=/usr/lib/ccache:$PATH" >> ~/.zshrc

	cd ~
	sudo apt-get install python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-lxml python3-pygame
	pip install PyYAML mavproxy --user
	echo "export PATH=$PATH:$HOME/.local/bin" >> ~/.bashrc
	pip install pandas
	pip install empy
	pip install opencv-python
	sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras
	wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
	sudo bash ./install_geographiclib_datasets.sh 
	
	copy folders in Files folder to catkin_ws/src folder
	cd ~/catkin_ws/
	catkin_make
	echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc
	echo "GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
	echo "export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins" >> ~/.bashrc
	echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/simple_example_description/models:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
	echo "export GAZEBO_MODEL_PATH=~/catkin_ws/src/simple_example_description/models_gazebo:${GAZEBO_MODEL_PATH}" >> ~/.bashrc
	echo "export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/simple_example_description/worlds:${GAZEBO_RESOURCE_PATH}" >> ~/.bashrc
	echo "export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/simple_example_description/build:${GAZEBO_PLUGIN_PATH}" >> ~/.bashrc
		
*****************
COMMANDS:

	1- roslaunch gazebo_ros empty_world.launch world_name:=zephyr_ardupilot_demo.world
	2- in second terminal:
		cd ~/ardupilot/ArduPlane
		sim_vehicle.py -f gazebo-zephyr --console --map
	3- in third terminal:
		roslaunch mavros apm.launch fcu_url:="tcp://127.0.0.1:5762"
	4- in fourth terminal:
		roslaunch launching launch.launch 
	so far we can manually control the plane using arrow keys in keyboard
	but your mouse pointer must be in fourth terminal
	and if you want to start self tracking go to step 5
