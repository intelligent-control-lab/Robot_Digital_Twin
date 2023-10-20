# Robot_Digital_Twin
This repository includes the digital twin models for (1) Fanuc LR-mate 200id-7L, (2) Yaskawa GP4 and (3) Lego bricks.

## Rviz visualization
```
roslaunch robot_digital_twin fanuc_rviz.launch
// OR
roslaunch robot_digital_twin gp4_rviz.launch
```

## Gazebo Usage
1. Create your digital twin launch file.
2. Launch the Gazebo world.
```
roslaunch robot_digital_twin your_launch_file.launch
```

### Examples:
1. Single Fanuc arm environment with Lego bricks.
```
roslaunch robot_digital_twin single_fanuc_lego.launch
```
2. Dual GP4 arms environment.
```
roslaunch robot_digital_twin dual_gp4.launch
```