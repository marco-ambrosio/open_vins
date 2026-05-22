from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

launch_args = [
    DeclareLaunchArgument(name="namespace", default_value="ov_msckf", description="namespace"),
    DeclareLaunchArgument(
        name="config",
        default_value="tum_vi",
        description="euroc_mav, tum_vi, rpng_aruco, kaist",
    ),
    DeclareLaunchArgument(
        name="config_path",
        default_value="",
        description="path to estimator_config.yaml. If not given, determined from 'config'",
    ),
    DeclareLaunchArgument(
        name="verbosity",
        default_value="INFO",
        description="ALL, DEBUG, INFO, WARNING, ERROR, SILENT",
    ),
    DeclareLaunchArgument(
        name="max_cameras",
        default_value="2",
        description="how many cameras we have 1 = mono, 2 = stereo, >2 = binocular",
    ),
    DeclareLaunchArgument(
        name="use_stereo",
        default_value="true",
        description="if we should track stereo constraints between synchronized pairs",
    ),
    DeclareLaunchArgument(
        name="bag_start",
        default_value="0.0",
        description="bag offset in seconds",
    ),
    DeclareLaunchArgument(
        name="bag_durr",
        default_value="-1.0",
        description="bag duration in seconds, <0 plays to the end",
    ),
    DeclareLaunchArgument(
        name="dataset",
        default_value="dataset-room1_512_16",
        description="dataset name used to derive default bag and groundtruth paths",
    ),
    DeclareLaunchArgument(
        name="dataset_root",
        default_value=EnvironmentVariable("OPENVINS_DATASET_DIR", default_value=""),
        description="root directory containing rosbag2 datasets used to derive the default bag path",
    ),
    DeclareLaunchArgument(
        name="bag",
        default_value="",
        description="path to rosbag2 bag directory",
    ),
    DeclareLaunchArgument(
        name="bag_storage_id",
        default_value="sqlite3",
        description="rosbag2 storage backend",
    ),
    DeclareLaunchArgument(
        name="bag_serialization_format",
        default_value="cdr",
        description="rosbag2 serialization format",
    ),
    DeclareLaunchArgument(
        name="dosave",
        default_value="false",
        description="save estimated trajectory to file (requires ov_eval pose_to_file node)",
    ),
    DeclareLaunchArgument(
        name="path_est",
        default_value="/tmp/traj_estimate.txt",
        description="output path for estimated trajectory",
    ),
    DeclareLaunchArgument(
        name="dotime",
        default_value="false",
        description="record timing information",
    ),
    DeclareLaunchArgument(
        name="path_time",
        default_value="/tmp/traj_timing.txt",
        description="timing statistics output path",
    ),
    DeclareLaunchArgument(
        name="dolivetraj",
        default_value="false",
        description="visualize live aligned GT trajectory (requires ov_eval live_align_trajectory node)",
    ),
    DeclareLaunchArgument(
        name="path_gt",
        default_value="",
        description="optional ASL groundtruth csv path",
    ),
]


def launch_setup(context):
    package_share = get_package_share_directory("ov_msckf")

    config_path = LaunchConfiguration("config_path").perform(context)
    if not config_path:
        configs_dir = os.path.join(package_share, "config")
        available_configs = os.listdir(configs_dir)
        config = LaunchConfiguration("config").perform(context)
        if config in available_configs:
            config_path = os.path.join(package_share, "config", config, "estimator_config.yaml")
        else:
            return [
                LogInfo(
                    msg="ERROR: unknown config: '{}' - Available configs are: {} - not starting OpenVINS".format(
                        config, ", ".join(available_configs)
                    )
                )
            ]
    elif not os.path.isfile(config_path):
        return [
            LogInfo(
                msg="ERROR: config_path file: '{}' - does not exist. - not starting OpenVINS".format(
                    config_path
                )
            )
        ]

    bag_path = LaunchConfiguration("bag").perform(context)
    if not bag_path:
        dataset_root = LaunchConfiguration("dataset_root").perform(context)
        config = LaunchConfiguration("config").perform(context)
        dataset = LaunchConfiguration("dataset").perform(context)
        if not dataset_root:
            return [
                LogInfo(
                    msg="ERROR: bag was not provided and dataset_root/OPENVINS_DATASET_DIR is unset - not starting OpenVINS"
                )
            ]
        bag_path = os.path.join(dataset_root, config, dataset)

    namespace = LaunchConfiguration("namespace")

    return [
        Node(
            package="ov_msckf",
            executable="run_serial_msckf",
            namespace=namespace,
            output="screen",
            parameters=[
                {"verbosity": LaunchConfiguration("verbosity")},
                {"config_path": config_path},
                {"use_stereo": LaunchConfiguration("use_stereo")},
                {"max_cameras": LaunchConfiguration("max_cameras")},
                {"path_bag": bag_path},
                {"bag_start": LaunchConfiguration("bag_start")},
                {"bag_durr": LaunchConfiguration("bag_durr")},
                {"bag_storage_id": LaunchConfiguration("bag_storage_id")},
                {"bag_serialization_format": LaunchConfiguration("bag_serialization_format")},
                {"record_timing_information": LaunchConfiguration("dotime")},
                {"record_timing_filepath": LaunchConfiguration("path_time")},
                {"path_gt": LaunchConfiguration("path_gt")},
            ],
        ),
        # Record estimated trajectory to file (requires ov_eval to be built for ROS2)
        GroupAction(
            condition=IfCondition(LaunchConfiguration("dosave")),
            actions=[
                Node(
                    package="ov_eval",
                    executable="pose_to_file",
                    name="recorder_estimate",
                    output="screen",
                    parameters=[
                        {"topic": ["/", namespace, "/poseimu"]},
                        {"topic_type": "PoseWithCovarianceStamped"},
                        {"output": LaunchConfiguration("path_est")},
                    ],
                ),
            ],
        ),
        # Live groundtruth alignment visualization (requires ov_eval to be built for ROS2)
        GroupAction(
            condition=IfCondition(LaunchConfiguration("dolivetraj")),
            actions=[
                Node(
                    package="ov_eval",
                    executable="live_align_trajectory",
                    name="live_align_trajectory",
                    output="log",
                    parameters=[
                        {"alignment_type": "posyaw"},
                        {"path_gt": LaunchConfiguration("path_gt")},
                    ],
                ),
            ],
        ),
    ]


def generate_launch_description():
    opfunc = OpaqueFunction(function=launch_setup)
    ld = LaunchDescription(launch_args)
    ld.add_action(opfunc)
    return ld
