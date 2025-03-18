from launch import LaunchDescription, LaunchContext
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
    LaunchConfiguration,
)

from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz for the gripper visualization.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="true",
            description="Use fake hardware for preview.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="",
            description="transforms prefix",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context: LaunchContext) -> list[IncludeLaunchDescription]:
    # tf_prefix is implicitly used in robot_state_publisher (in URDF substitution)
    tf_prefix = LaunchConfiguration("tf_prefix", default="")  # noqa: F841

    return [
        preapre_control_node(),
        prepare_robot_state_publisher_node(),
        prepare_rviz_node(enable=LaunchConfiguration("launch_rviz")),
        prepare_controller_spawner("joint_state_broadcaster"),
        prepare_controller_spawner("gripper_action_controller"),
    ]


def prepare_rviz_node(enable: LaunchConfiguration):
    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("robotiq_hande_driver"),
            "bringup",
            "config",
            "rviz_preview.rviz",
        ]
    )
    return Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(enable),
    )


def preapre_control_node() -> Node:
    robot_controllers = ParameterFile(
        PathJoinSubstitution(
            [
                FindPackageShare("robotiq_hande_driver"),
                "bringup",
                "config",
                "hande_controller.yaml",
            ]
        ),
        allow_substs=True,
    )

    return Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
        output="both",
    )


def prepare_robot_state_publisher_node() -> Node:
    # tf_prefix is implicitly used in ParameterValue()
    tf_prefix = LaunchConfiguration("tf_prefix", default="")  # noqa: F841
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    robot_description_str = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robotiq_hande_description"),
                    "urdf",
                    "robotiq_hande_gripper.urdf.xacro",
                ]
            ),
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
        ]
    )
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": ParameterValue(robot_description_str, value_type=str)}
        ],
    )


def prepare_controller_spawner(controller_name: str) -> Node:
    return Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            controller_name,
            "--controller-manager",
            "/controller_manager",
        ],
    )
