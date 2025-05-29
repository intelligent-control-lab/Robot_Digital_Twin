# Robot_Digital_Twin
This repository includes the digital twin models for
* Fanuc LR-mate 200id-7L.
* Yaskawa GP4.
* Yaskawa GP50.
* Lego bricks, including Lego baseplates, 1x1, 1x2, 1x4, 1x6, 1x8, 2x2, 2x4, 2x6, 2x8.

## Robot Rviz visualization
```
roslaunch robot_digital_twin YOUR_ROBOT_rviz.launch
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

## Lego Visualization (single scene)
1. Specify the task json under gazebo/scripts/task
2. Launch the environment
```
roslaunch robot_digital_twin dual_gp4.launch
```
and the camera server
```
cd gazebo
python3 ./scripts/camera_server.py
```
3. Run script
```
python3 ./scripts/vis_lego.py --task 0000
```
where 0000 is the task name of the json file specification

## Lego Visualization (entire assembly sequence)
Script to visualize each assembly step for a given assembly sequence
```
python3 ./scripts/vis_assembly_seq.py --base_dir {path to assembly seq folder} --task {name of assembly}
```

## Citation
If you find this repository helpful, please kindly cite our work.
```
@article{liu2023simulationaided,
  title={Simulation-aided Learning from Demonstration for Robotic LEGO Construction}, 
  author={Ruixuan Liu and Alan Chen and Xusheng Luo and Changliu Liu},
  journal={arXiv preprint arXiv:2309.11010},
  year={2023}
}
```

## License
This project is licensed under the MIT License.
