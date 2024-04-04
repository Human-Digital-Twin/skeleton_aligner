# Skeleton Aligner

Align the body poses of a person estimated using [Xsens MVN Analyze](https://www.movella.com/products/motion-capture/mvn-analyze) to the ones estimated by [Microsoft Azure Kinect Body Tracking](https://learn.microsoft.com/en-us/azure/kinect-dk/body-sdk-setup).


## Dependencies
* [Hi-ROS Skeleton Messages](https://github.com/hiros-unipd/skeleton_msgs)


## Parameters
| Parameter            | Description                                                                   |
| -------------------- | ----------------------------------------------------------------------------- |
| `kinect_input_topic` | Input StampedSkeleton topic published by Azure Kinect Body Tracking SDK       |
| `xsens_input_topic`  | Input StampedSkeleton topic published by MVN Analyze                          |
| `output_topic`       | Output StampedSkeleton topic containing the aligned skeleton                  |
| `publish_tfs`        | Set if body poses should be published as ROS transforms                       |
| `config/marker_ids`  | Marker IDs to use to align translation and rotation between the two skeletons |


## Usage
```
ros2 launch skeleton_aligner default.launch.py
```


## Example Full Pipeline
Kinect:
1. launch [`azure_kinect_ros_driver`]((https://github.com/microsoft/Azure_Kinect_ROS_Driver)): publish body poses as MarkerArray messages
2. launch [`k4abt_converter`](https://github.com/HiROS-unipd/k4abt_converter): convert MarkerArray messages to SkeletonGroup messages
3. launch [`skeleton_publisher`](https://github.com/mguidolin/skeleton_publisher): extract a single StampedSkeleton from the SkeletonGroup

Xsens:
1. launch [`xsens_mvn_driver`](https://github.com/WEM-Platform/xsens_mvn_driver): publish body poses as LinearSegmentKinematics messages
2. launch [`mvn_converter`](https://github.com/mguidolin/mvn_converter): convert LinearSegmentKinematics messages to StampedSkeleton messages

Aligner:
1. launch [`skeleton_aligner`](https://github.com/mguidolin/skeleton_aligner): align the two skeletons
