#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_ros
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

# if is a XACRO file, disocment this line
import xacro

def generate_launch_description():

    # True if the robot file is a Xacro
    IsXacro = False
    # robotName -> this name must bem the same as in the URDF ou XACRO robot nema tag (<robot name="go2"> [line 6 of the urdf file])
    robotName ='go2'
    # pkgName -> name of the package
    pkgName = 'go2_description'
    # model relative path
    modelFileRelativePath = 'urdf/go2.urdf'
    # world relative path
    worldFileRelativePath = 'worlds/empty_world.world'

    # create the package path 
    pkgPath = launch_ros.substitutions.FindPackageShare(package=pkgName).find(pkgName)
    # path to the URDF/XACRO 
    modelPath = os.path.join(pkgPath, modelFileRelativePath)
    # path to the world file
    pathWorldFile = os.path.join(pkgPath, worldFileRelativePath)
    # get the path to the ros_gz package 
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # create the robot description
    if IsXacro:
        robot_desc = xacro.process_file(modelPath).toxml()
    else:
        with open(modelPath,'r') as infp:
            robot_desc = infp.read()

    # create a dict with the robot_desc
    params ={'robot_description':robot_desc}

    # create a node of the that publishes the robot state
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params],
        arguments='modelPath'
    )
    
    # include the world file to launch
    gzh_rosPackagelaunch = PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch','gz_sim.launch.py'))
    gzh_launch = IncludeLaunchDescription(gzh_rosPackagelaunch, launch_arguments={'gz_args': [pathWorldFile]}.items())

    # create the node that spaws the robot
    gzh_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robotName,
            '-file', modelPath,
            '-x', '0',
            '-y', '0',
            '-z', '0.6'
        ],
        output='screen',
    )
    
    # this code ensures that gazebo finds the robot parts
    if "GZ_SIM_RESOURCE_PATH" in os.environ:
        gz_sim_resource_path = os.environ["GZ_SIM_RESOURCE_PATH"]

        if "SDF_PATH" in os.environ:
            sdf_path = os.environ["SDF_PATH"]
            os.environ["SDF_PATH"] = sdf_path + ":" + gz_sim_resource_path
        else:
            os.environ["SDF_PATH"] = gz_sim_resource_path    

    # create the rviz node 
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen'
    #     )

    # create the launch description
    launchDescriptionObject = LaunchDescription()

    # add the the nodes to the launch description
    launchDescriptionObject.add_action(gzh_launch)
    launchDescriptionObject.add_action(robot_state_publisher_node)
    launchDescriptionObject.add_action(gzh_spawner_cmd)
    # launchDescriptionObject.add_action(rviz_node)

    # return the launch description
    return launchDescriptionObject