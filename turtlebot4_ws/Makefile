
build-pack:
	@echo "Building the turtlebot4 simulation"
	bash -c "source /opt/ros/humble/setup.bash &&\
	colcon build --symlink-install"

run-sim:
	@echo "Running the turtlebot4 simulation with custom maze world and scanned map"
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