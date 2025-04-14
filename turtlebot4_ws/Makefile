
build-pack:
	@echo "Building the turtlebot4 simulation"
	bash -c "source /opt/ros/humble/setup.bash &&\
	colcon build --symlink-install"

run-sim-boxes:
	@echo "Running the turtlebot4 simulation with lab 211 with boxes and scanned map"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
	world:=lab211_with_boxes_s_1 \
	map:=/home/eason-pan/Documents/Gits/5335-final-project/turtlebot4_ws/maps/lab211_with_boxes.yaml \
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

run-mapping:
	@echo "Running the turtlebot4 simulation with lab 211 to map the environment"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
	world:=lab211_with_boxes_s_1 \
	model:=lite \
	nav2:=false \
	rviz:=true \
	slam:=true \
	localization:=false"

launch-node-adj:
	@echo "Launching the node to adjust the map"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_safety safety_params.launch.py"