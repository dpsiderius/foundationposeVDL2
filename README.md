# FoundationPose VDL2

This repository contains a ROS2 workspace for object pose estimation using FoundationPose and Isaac ROS, with custom perception components for Raspberry Pi integration.

## Structure

- `src/raspberry_perception/` - Custom perception package for object tracking and manual selection
- `src/isaac_ros_common/` - Isaac ROS common utilities (submodule)
- `src/isaac_ros_pose_estimation/` - Isaac ROS pose estimation packages (submodule)

## Dependencies

This project uses NVIDIA Isaac ROS packages as git submodules. The main dependencies are:

- Isaac ROS Common
- Isaac ROS Pose Estimation (includes FoundationPose, DOPE, CenterPose)

## Setup

1. Clone this repository with submodules:
```bash
git clone --recursive https://github.com/dpsiderius/foundationposeVDL2.git
cd foundationposeVDL2
```

2. Build the workspace:
```bash
colcon build --symlink-install
```

3. Source the workspace:
```bash
source install/setup.bash
```

## Components

### Raspberry Perception Manager

The `manager_node.py` implements a state machine for object tracking:

- **IDLE**: Waiting for action goal
- **AWAITING_SELECTION**: Waiting for user to select ROI
- **SEGMENTING**: Processing selected region
- **TRACKING_POSE**: Actively tracking object pose

### Services and Topics

- Action: `track_object` - Start object tracking
- Service: `/reset_selection` - Reset to selection state
- Subscriptions: `/object_pose`, `/selected_roi`

## Usage

Launch the perception manager:
```bash
ros2 run raspberry_perception manager_node
```

## License

This project combines components under different licenses:
- Custom code: [Your License]
- Isaac ROS components: Apache 2.0 (NVIDIA) 