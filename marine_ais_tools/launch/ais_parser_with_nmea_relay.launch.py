from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    config = PathJoinSubstitution([
        get_package_share_directory("marine_ais_tools"),
        "config",
        "parameters.yaml"
    ])

    nmea_relay = Node(
        package="marine_ais_tools",
        executable="nmea_relay",
        name="nmea_relay",
        remappings=[
            ("/nmea", "/ais/raw"),
        ],
        parameters=[config]
    )

    ais_parser = Node(
        package="marine_ais_tools",
        executable="ais_parser",
        name="ais_parser",
        remappings=[
            ("/nmea", "/ais/raw"),
            ("/messages", "/ais/messages"),
        ],
    )

    return LaunchDescription([
        nmea_relay,
        ais_parser
    ])