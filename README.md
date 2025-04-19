# 5335-final-project
Keep the robot safe while travesing through mazes. 

## Stages and progress
| Stage | Simulation Progress | Physical Robot Progress |
| ----- | ------------------- | ----------------------- |
| S1 - all obstacles mapped out | 100% - implemented, video recorded | N/A |
| S2 - no obstacles mapped out | 100% - implemented, video recorded | to be tested |
| S3 - have fiducial codes guiding the robot | implemented, can't test in simulation | to be tested |


## Installation and Running
### System Requirements
- Make sure ROS2 is installed
- For simulation runs, make sure Gazebo Simulator, Ignition Fortress are installed
- cd into turtlebot4_ws directory for all commands below

### Installation
- To clean up installs/builds/logs: `make clean-up`
- To build all packages: `make build-all`
- To only build the core project (without simulator packages): `make build-safety`

### Running
#### Run with Physical Turtlebot4 Lite
- Stage 2: `make run-s2`
- Stage 3: `make run-s3`
#### Run with Simulation
- Stage 1:
    - Terminal 1: `make run-sim-boxes`
    - Terminal 2: `make launch-node-adj`
- Stage 2: `make run-s2-sim`
- Stage 3: `make run-s3-sim`
#### Test virtual world and map setups
- Map with boxes: `make run-sim-boxes`
- Map without boxes: `make run-sim-empty`
- To map the environment: `make run-mapping`

### When Gazebo Simulator and Rivz started
- create a starting pose in Rivz matching the robot's starting pose
- then you will be able to set goal pose and let it navigate through to the goal pose