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
