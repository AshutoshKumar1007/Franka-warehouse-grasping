import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    share = get_package_share_directory('franka_perception')
    cfg_path = context.perform_substitution(LaunchConfiguration('camera_config'))
    with open(cfg_path) as f:
        cfg = yaml.safe_load(f)

    cam = cfg['camera']
    perc = cfg['perception']
    x, y, z = cam['position']['x'], cam['position']['y'], cam['position']['z']
    roll, pitch, yaw = (cam['orientation_rpy']['roll'],
                        cam['orientation_rpy']['pitch'],
                        cam['orientation_rpy']['yaw'])

    world_name = context.perform_substitution(LaunchConfiguration('world_name'))
    camera_sdf = os.path.join(share, 'models', 'overhead_camera.sdf')

    spawn_camera = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-world', world_name, '-file', camera_sdf, '-x', str(x), '-y', str(y), '-z', str(z)],
        output='screen',
    )

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        arguments=[
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
    )

    static_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        arguments=['--x', str(x), '--y', str(y), '--z', str(z),
                   '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
                   '--frame-id', cam['parent_frame'], '--child-frame-id', cam['camera_frame']],
        output='screen',
    )

    perception_node = Node(
        package='franka_perception', executable='perception_node',
        parameters=[{
            'input_prefix': perc['input_prefix'],
            'output_prefix': perc['output_prefix'],
            'target_frame': perc['target_frame'],
        }],
        output='screen',
    )

    return [spawn_camera, bridge, static_tf, perception_node]


def generate_launch_description():
    default_cfg = os.path.join(
        get_package_share_directory('franka_perception'), 'config', 'camera.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('world_name', default_value='warehouse_box_large',
                               description='Gazebo world name to spawn the camera into'),
        DeclareLaunchArgument('camera_config', default_value=default_cfg,
                               description='Path to camera.yaml'),
        OpaqueFunction(function=launch_setup),
    ])
