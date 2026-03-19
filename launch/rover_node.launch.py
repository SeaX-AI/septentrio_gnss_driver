#!/usr/bin/env python3
"""
Septentrio GNSS/INS Driver Launch File
=======================================

Launches the Septentrio GNSS/INS driver node and the vehicle state fusion node.
Configuration is loaded from YAML files with DEV/PROD mode support:
  - DEV mode:  Uses package share directory configs
  - PROD mode: Uses $CONFIG_DIR environment variable if set

Configuration files managed:
  1. rover_node.yaml - Main GNSS driver configuration (device, protocol, publish settings)
  2. sens_fusion.yaml - Sensor fusion configuration (frame IDs, IMU calibration, origin)

Parameters:
  - circuit_name: Map/scenario name used to load origin coordinates from JSON
                  (Torremocha, aeropuerto, PoligonoTeruel, ...)
  - use_sim_time: Use simulation time (true) or wall clock time (false)
                  true: rosbag replay
                  false: Real sensor (default)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

os.environ['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '{time}: [{name}] [{severity}]\t{message}'

# Mapping from common circuit names (used in scripts/stack launches) to
# JSON scenario keys in Tracks_main_info.json (case-sensitive)
CIRCUIT_TO_SCENARIO = {
    'torremocha':                       'Torremocha',
    'Torremocha':                       'Torremocha',
    'aeropuerto':                       'Aeropuerto',
    'Aeropuerto':                       'Aeropuerto',
    'poligono_teruel':                  'TeruelPoligono',
    'PoligonoTeruel':                   'TeruelPoligono',
    'TeruelPoligono':                   'TeruelPoligono',
    'sendaviva_tudela_parking':         'SendaViva_Parking',
    'sendaviva_tudela_parking_reverse': 'SendaViva_Parking',
    'SendaViva_Parking':                'SendaViva_Parking',
    'sendaviva_tudela_entrada':         'SendaViva',
    'SendaViva':                        'SendaViva',
    'MotorLand':                        'MotorLand',
}


def launch_setup(context, *args, **kwargs):
    """Setup function to configure nodes at runtime (resolves LaunchConfigurations)."""

    circuit_name = LaunchConfiguration('circuit_name').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)

    # Resolve circuit name to JSON scenario key (fallback: pass as-is)
    selected_scenario = CIRCUIT_TO_SCENARIO.get(circuit_name, circuit_name)

    pkg_share = get_package_share_directory('septentrio_gnss_driver')
    pkg_config_dir = os.path.join(pkg_share, 'config')

    # Path to the scenarios JSON file (always from package share)
    scenarios_json_path = os.path.join(pkg_config_dir, 'Tracks_main_info.json')

    # Check for external config directory (PROD mode)
    config_dir = os.environ.get('CONFIG_DIR', '')

    # ===== CONFIG FILE 1: rover_node.yaml =====
    rover_config = os.path.join(pkg_config_dir, 'rover_node.yaml')
    if config_dir:
        external_rover = os.path.join(config_dir, 'rover_node.yaml')
        if os.path.exists(external_rover):
            rover_config = external_rover
            print(f"[PROD MODE] rover_node.yaml  → {rover_config}", flush=True)
        else:
            print(f"[DEV  MODE] rover_node.yaml  → {rover_config} (CONFIG_DIR set but file not found)", flush=True)
    else:
        print(f"[DEV  MODE] rover_node.yaml  → {rover_config}", flush=True)

    # ===== CONFIG FILE 2: sens_fusion.yaml =====
    fusion_config = os.path.join(pkg_config_dir, 'sens_fusion.yaml')
    if config_dir:
        external_fusion = os.path.join(config_dir, 'sens_fusion.yaml')
        if os.path.exists(external_fusion):
            fusion_config = external_fusion
            print(f"[PROD MODE] sens_fusion.yaml → {fusion_config}", flush=True)
        else:
            print(f"[DEV  MODE] sens_fusion.yaml → {fusion_config} (CONFIG_DIR set but file not found)", flush=True)
    else:
        print(f"[DEV  MODE] sens_fusion.yaml → {fusion_config}", flush=True)

    print(f"   circuit_name     : {circuit_name}", flush=True)
    print(f"   selected_scenario: {selected_scenario}", flush=True)
    print(f"   scenarios_json   : {scenarios_json_path}", flush=True)
    print(f"   use_sim_time     : {use_sim_time}", flush=True)
    print("=" * 70 + "\n", flush=True)

    # ===== STATIC TF NODES =====
    tf_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 base_link imu".split(' ')
    )
    tf_gnss = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 imu gnss".split(' ')
    )
    tf_vsm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 imu vsm".split(' ')
    )
    tf_aux1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0 0 0 0 0 0 imu aux1".split(' ')
    )

    # ===== GNSS/INS DRIVER NODE =====
    gnss_node = Node(
        package='septentrio_gnss_driver',
        executable='septentrio_gnss_driver_node',
        name='septentrio_gnss_driver',
        emulate_tty=True,
        sigterm_timeout='20',
        parameters=[
            rover_config,
            {
                'use_sim_time': use_sim_time == 'true',
            }
        ],
        remappings=[
            ('navsatfix',     '/septentrio/navsatfix'),
            ('gpsfix',        '/septentrio/gpsfix'),
            ('pvtgeodetic',   '/septentrio/pvtgeodetic'),
            ('poscovgeodetic','/septentrio/poscovgeodetic'),
            ('velcovgeodetic','/septentrio/velcovgeodetic'),
            ('insnavgeod',    '/septentrio/insnavgeod'),
            ('imu',           '/septentrio/imu'),
            ('pose',          '/septentrio/pose'),
            ('twist',         '/septentrio/twist'),
            ('twist_gnss',    '/septentrio/twist_gnss'),
            ('twist_ins',     '/septentrio/twist_ins'),
            ('diagnostics',   '/septentrio/diagnostics'),
            ('aimplusstatus', '/septentrio/aimplusstatus'),
            ('localization',  '/septentrio/localization'),
            ('atteuler',      '/septentrio/atteuler'),
            ('attcoveuler',   '/septentrio/attcoveuler'),
            ('extsensormeas', '/septentrio/extsensormeas'),
        ]
    )

    # ===== VEHICLE STATE FUSION NODE =====
    fusion_node = Node(
        package='septentrio_gnss_driver',
        executable='vehicle_state_fusion_node',
        name='vehicle_state_fusion',
        output='screen',
        parameters=[
            fusion_config,
            {
                # Override scenario and JSON path from launch argument
                'selected_scenario':   selected_scenario,
                'scenarios_json_path': scenarios_json_path,
                'use_sim_time':        use_sim_time == 'true',
            }
        ]
    )

    return [tf_imu, tf_gnss, tf_vsm, tf_aux1, gnss_node, fusion_node]


def generate_launch_description():
    """Generate launch description for Septentrio GNSS driver with DEV/PROD config support."""

    print("\n" + "=" * 70, flush=True)
    print("🛰️  SEPTENTRIO GNSS/INS - Configuration Loading", flush=True)
    print("=" * 70, flush=True)

    circuit_name_arg = DeclareLaunchArgument(
        'circuit_name',
        default_value='aeropuerto',
        description='Circuit/map name: Torremocha, aeropuerto, PoligonoTeruel, ...'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time (true) or wall clock time (false)'
    )

    return LaunchDescription([
        circuit_name_arg,
        use_sim_time_arg,
        OpaqueFunction(function=launch_setup),
    ])
