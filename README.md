# Skeleton Aligner

Align the body poses of a person expressed as [StampedSkeleton messages](https://github.com/hiros-unipd/skeleton_msgs).


## Dependencies
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)


## Parameters
| Parameter            | Description                                                                    |
| -------------------- | ------------------------------------------------------------------------------ |
| `input_topics`       | Vector of input StampedSkeleton topics to align (**currently limited to 2**)   |
| `weight`             | Value to use to calculate the weighted least squares average of the transforms |
| `config/marker_ids`  | Marker IDs to use to align translation and rotation between the two skeletons  |


## Usage
```
ros2 launch skeleton_aligner default.launch.py
```

## Citation
Please cite the following paper:
```
M. Guidolin, N. Berti, P. Gnesotto, D. Battini and M. Reggiani, "Trust the Robot! Enabling Flexible Collaboration With Humans via Multi-Sensor Data Integration," 2024 IEEE 29th International Conference on Emerging Technologies and Factory Automation (ETFA), Padova, Italy, 2024, pp. 1-8, doi: 10.1109/ETFA61755.2024.10710349.
```

Bib citation source:
```bibtex
@INPROCEEDINGS{10710349,
  author={Guidolin, Mattia and Berti, Nicola and Gnesotto, Paride and Battini, Daria and Reggiani, Monica},
  booktitle={2024 IEEE 29th International Conference on Emerging Technologies and Factory Automation (ETFA)}, 
  title={Trust the Robot! Enabling Flexible Collaboration With Humans via Multi-Sensor Data Integration}, 
  year={2024},
  volume={},
  number={},
  pages={1-8},
  keywords={Pose estimation;Collaborative robots;Sensor fusion;Robot sensing systems;Real-time systems;Workstations;Safety;Reliability;Monitoring;Intelligent sensors;Human Digital Twin;Human-Robot Collaboration;Sensor Fusion},
  doi={10.1109/ETFA61755.2024.10710349}}
```
