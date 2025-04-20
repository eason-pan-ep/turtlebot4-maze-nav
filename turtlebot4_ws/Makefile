########################################################################
### all commends require cd to turtlebot4_ws and have ROS2 installed ###
########################################################################

### clean-up all###
clean-up:
	@echo "Cleaning up all install/build/log folders"
	rm -rf build install log
### install required Python packages ###
python-setup:
	@echo "Installing required Python packages"
	bash -c "pip install -r requirements.txt"
### build all packages including simulation packages ###
build-all:
	@echo "Building the turtlebot4 simulation"
	bash -c "source /opt/ros/humble/setup.bash &&\
	colcon build --symlink-install"
### only build turtlebot4_safety ###
build-safety:
	@echo "Building the turtlebot4_safety package"
	bash -c "source /opt/ros/humble/setup.bash &&\
	colcon build --packages-select turtlebot4_safety"

### Runs for physical robot ###
run-s2:
	@echo "Running S2 the turtlebot4 physical robot"
	@echo "Ctrl+C to stop\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_safety slam_nav_physical.launch.py"

run-s3:
	@echo "Running S3 the turtlebot4 physical robot"
	@echo "Ctrl+C to stop\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_safety slam_navigation_AR.launch.py"

### Runs for simulation ###
run-s2-sim:
	@echo "Running S2 the turtlebot4 simulation"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_safety slam_with_room_navigation.launch.py"

run-s3-sim:
	@echo "Running S3 the turtlebot4 simulation"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_safety slam_nav_AR_sim.launch.py"

### below are 2 runs for testing simulation works with virtual lab211 with boxes, 
### and map with only walls or with boxes ###
run-sim-boxes:
	@echo "Running the turtlebot4 simulation with lab 211 with boxes and scanned map"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
	world:=lab211_with_boxes_1 \
	map:=/home/eason-pan/Documents/Gits/5335-final-project/turtlebot4_ws/maps/lab211_with_boxes_n.yaml \
	model:=lite \
	nav2:=true \
	rviz:=true \
	slam:=false \
	localization:=true"

run-sim-empty:
	@echo "Running the turtlebot4 simulation with empty lab 211 with boxes and scanned map"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
	world:=lab_s_1 \
	map:=/home/eason-pan/Documents/Gits/5335-final-project/turtlebot4_ws/maps/lab211_empty.yaml \
	model:=lite \
	nav2:=true \
	rviz:=true \
	slam:=false \
	localization:=true"

### help mapping the environment ###
run-mapping:
	@echo "Running the turtlebot4 simulation with lab 211 to map the environment"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
	world:=lab211_with_boxes_1 \
	model:=lite \
	slam:=true \
	nav2:=false \
	rviz:=true \
	localization:=false"

### Additional node for running with boxes mapped ###
launch-node-adj:
	@echo "Launching the node to adjust the map"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_safety safety_params.launch.py"