from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
import os
from pathlib import Path

def generate_launch_description():
    teleop = ExecuteProcess(
        name='teleop_keyboard',
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard',
            '--ros-args', '--remap', 'cmd_vel:=/dss_vehicle/cmd_vel'],
        shell=False,
    )

    # Launch a node
    dss_station = ExecuteProcess(
        name='dss_station',
        cmd=['ros2', 'run', 'dss_station', 'dss_station'],
        output='screen',
    )

    # Callback function when node exits
    def on_node_exit(event, context):
        return_code = event.returncode
        if return_code != 0:
            cleanup = ExecuteProcess(
                name='off_pwm',
                cmd=['bash', '-c', off_pwm_script],
                shell=True,
            )
            return [cleanup]
            shutdown = EmitEvent(event=Shutdown(reason='dss_station crashed'))  # Emit shutdown event (optional, stops the entire launch)
            return [cleanup, shutdown]

    # Register event handler
    on_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=dss_station,
            on_exit=on_node_exit
        )
    )

    return LaunchDescription([
        teleop,
        dss_station,
        # on_exit_handler
    ])
