
build:
	@echo "Building the turtlebot4 simulation"
	bash -c "source /opt/ros/humble/setup.bash" &&\
	colcon build --symlink-install

run:
	@echo "Running the turtlebot4 simulation with custom maze world and scanned map"
	@echo "Ctrl+C to stop the simulation\n"
	bash -c "source install/setup.bash && \
	ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py \
	world:=maze_t8 \
	map:=/home/eason-pan/Documents/Gits/5335-final-project/turtlebot4_ws/maps/maze_t8.yaml \
	model:=lite \
	nav2:=true \
	rviz:=true \
	slam:=false \
	localization:=true"