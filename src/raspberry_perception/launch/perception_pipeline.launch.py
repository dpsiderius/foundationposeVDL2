from launch import LaunchDescription
from launch_ros.actions import Node, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('realsense2_camera'),'launch','rs_launch.py'])
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'pointcloud.enable': 'false',
            'enable_infra1': 'false',
            'enable_infra2': 'false'
        }.items()
    )

    manual_sel = Node(package='raspberry_perception', executable='manual_selection_node',
                      name='manual_selection_node', output='screen')

    sam_seg = Node(package='raspberry_perception', executable='sam_segmentation_node',
                   name='sam_segmentation_node', output='screen')

    fp_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('isaac_ros_foundationpose'),
                'launch','isaac_ros_foundationpose_core.launch.py'
            ])
        ),
        launch_arguments={
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'mask_topic': '/segmentation/mask',
            'verbose': 'true'
            # Add *engine/model* params once assets are in place
        }.items()
    )

    manager = Node(package='raspberry_perception', executable='manager_node',
                   name='manager_node', output='screen')

    return LaunchDescription([realsense_launch, manual_sel, sam_seg, fp_core_launch, manager])
