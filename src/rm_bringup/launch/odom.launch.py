"""
odom.launch.py — the odometry chain, without the vendor base driver.

What it does:
    Starts the three vendor nodes that turn a wheel-velocity stream into
    /odom, and nothing else:

        rm_base /vel_raw      -> base_node_X3         -> /odom_raw
        rm_base /imu/data_raw -> imu_filter_madgwick  -> /imu/data
        /odom_raw + /imu/data -> robot_localization   -> /odom

    Plus robot_state_publisher / joint_state_publisher for the URDF TF tree
    the EKF resolves base_footprint against.

Why this file exists:
    The vendor `yahboomcar_bringup_X3_launch.py` bundles those nodes together
    with two we must NOT run:

      * `Mcnamu_driver_X3` — the base driver. rm_base REPLACES it and the
        base-board serial port is single-writer, so starting the vendor
        bringup while rm_base is up makes both read garbage.
      * `yahboom_joy_X3` — publishes /cmd_vel directly, which rm_safety alone
        is allowed to own. Use rm_bringup/launch/teleop_joy.launch.py, which
        runs the same node remapped to /teleop/cmd_vel, i.e. as an INPUT to
        rm_safety.

    The vendor launch has no argument to disable either, so this file starts
    the wanted subset directly. It is the wiring BUILD-PLAN "next session"
    item 5 called for: rm_base /vel_raw -> the yahboom odom integrator -> EKF.

Subscribes / publishes:
    consumes /vel_raw, /imu/data_raw (both from rm_base_node)
    produces /odom_raw, /imu/data, /odom, and the odom -> base_footprint TF.

WHAT /odom IS AND IS NOT, HERE:
    /odom is now derived from rm_base's RECONSTRUCTED /vel_raw, which is
    itself unverified against a tape measure (BUILD-PLAN item 4). Treat it as
    a LIVENESS signal, not a measurement:

      * rm_safety only needs it to be fresh — safety_node._odom_cb
        deliberately does NOT feed odom velocity into EvalInputs, it only
        stamps the staleness watchdog. So odom accuracy cannot change a
        safety decision, and running this launch is what clears the
        `odom_stale` stop condition.
      * policy_node DOES derive pose and goal from /odom. Those remain
        untrustworthy until /vel_raw is validated. Do not build a
        heading-hold controller on this.

Launch arguments:
    pub_odom_tf   "false" (default, matches the vendor). The EKF publishes
                  the odom -> base_footprint transform; letting the
                  integrator publish it too gives two writers on one TF.

Failure mode:
    Passive. Every node here is a consumer or a filter — none of them can
    command the base. If rm_base_node is not running, /vel_raw never arrives,
    /odom goes stale, and rm_safety fails closed to a stop condition, which
    is the intended direction.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("yahboomcar_description")
    default_model_path = os.path.join(
        description_share, "urdf", "yahboomcar_X3.urdf"
    )

    # Vendor's own madgwick tuning — reused verbatim so /imu/data matches what
    # the EKF was configured against.
    imu_filter_config = os.path.join(
        get_package_share_directory("yahboomcar_bringup"),
        "param",
        "imu_filter_param.yaml",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]), value_type=str
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "model", default_value=default_model_path,
            description="Absolute path to the robot URDF.",
        ),
        DeclareLaunchArgument(
            "pub_odom_tf", default_value="false",
            description="Let the odom integrator publish odom->base_footprint. "
                        "Leave false: the EKF already publishes it.",
        ),
        DeclareLaunchArgument(
            "gui", default_value="false",
            description="Use joint_state_publisher_gui instead of the "
                        "headless joint_state_publisher.",
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=UnlessCondition(LaunchConfiguration("gui")),
        ),

        # /vel_raw -> /odom_raw. Scales left at the vendor defaults of 1.0:
        # rm_base already applies the calibration, so scaling here too would
        # double-count it.
        Node(
            package="yahboomcar_base_node",
            executable="base_node_X3",
            parameters=[{
                "pub_odom_tf": LaunchConfiguration("pub_odom_tf"),
                "linear_scale_x": 1.0,
                "linear_scale_y": 1.0,
                "angular_scale": 1.0,
            }],
        ),

        # /imu/data_raw -> /imu/data
        Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            parameters=[imu_filter_config],
        ),

        # /odom_raw + /imu/data -> /odom
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(
                    get_package_share_directory("robot_localization"), "launch"
                ),
                "/ekf_x1_x3_launch.py",
            ])
        ),
    ])
