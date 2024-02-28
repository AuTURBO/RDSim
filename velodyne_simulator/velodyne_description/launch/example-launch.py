#*********************************************************************
 # Software License Agreement (BSD License)
 #
 #  Copyright (c) 2019, Open Source Robotics Foundation, Inc.
 #  All rights reserved.
 #
 #  Redistribution and use in source and binary forms, with or without
 #  modification, are permitted provided that the following conditions
 #  are met:
 #
 #   * Redistributions of source code must retain the above copyright
 #     notice, this list of conditions and the following disclaimer.
 #   * Redistributions in binary form must reproduce the above
 #     copyright notice, this list of conditions and the following
 #     disclaimer in the documentation and/or other materials provided
 #     with the distribution.
 #   * Neither the name of Dataspeed Inc. nor the names of its
 #     contributors may be used to endorse or promote products derived
 #     from this software without specific prior written permission.
 #
 #  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 #  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 #  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 #  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 #  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 #  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 #  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 #  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 #  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 #  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 #  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 #  POSSIBILITY OF SUCH DAMAGE.
 #********************************************************************

import os

from ament_index_python.packages import get_package_share_directory
import ament_index_python.packages
import launch
import launch_ros.actions
import subprocess

def read_file(path):
    with open(path, 'r') as f:
        contents = f.read()
    return contents

def generate_launch_description():
    output_mode = 'both'
    gpu = False;

    gazebo_dir = os.path.dirname(get_package_share_directory('velodyne_description'))
    world = os.path.join(get_package_share_directory('velodyne_description'),
        'world', 'example.world')

    urdf_dir =  os.path.join(get_package_share_directory('velodyne_description'),
        'urdf')
    xacro_urdf = os.path.join(urdf_dir, 'example.urdf.xacro')
    robot_urdf = os.path.join(urdf_dir, 'example.urdf')
    xacro_proc = subprocess.Popen("xacro {0} gpu:={1}  > {2}".format(xacro_urdf, gpu, robot_urdf) ,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, shell=True)
    xacro_proc.wait()
    assert os.path.exists(robot_urdf)
    urdf_contents = read_file(robot_urdf)

    rviz_config = os.path.join(
        ament_index_python.packages.get_package_share_directory('velodyne_description'),
        'rviz', 'example.rviz')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                              node_executable='robot_state_publisher',
                              output='both',
                              arguments=[robot_urdf])

    rviz = launch_ros.actions.Node(package='rviz2',
                              node_executable='rviz2',
                              output='both',
                              arguments=['-d', rviz_config])

    spawn_entity_message_contents = "'{initial_pose:{ position: {x: 0, y: 0, z: 0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}},  name: \"velodyne_description\", xml: \"" + urdf_contents.replace('"', '\\"') + "\"}'"
    spawn_entity = launch.actions.ExecuteProcess(
        name='spawn_entity', cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_entity_message_contents], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)

    # Publishes the URDF to /robot_description.
    # This is a workaround to make rviz get the urdf.
    urdf_pub_data = urdf_contents.replace('"', '\\"')
    launch_urdf = launch.actions.ExecuteProcess(
        name='launch_urdf', cmd=['ros2', 'topic', 'pub', '-r', '0.1', '/robot_description', 'std_msgs/String', '\'data: "' + urdf_pub_data + '"\''], env=os.environ.copy(), output=output_mode, shell=True, log_cmd=False)

    my_env = os.environ.copy()
    my_env["GAZEBO_MODEL_PATH"] = gazebo_dir
    gazebo = launch.actions.ExecuteProcess(
        name='gazebo', cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', world], env=my_env, output=output_mode, shell=True, log_cmd=False)

    return launch.LaunchDescription([rsp,
                                     rviz,
                                     gazebo,
                                     spawn_entity,
                                     launch_urdf,
                                     launch.actions.RegisterEventHandler(
                                      event_handler=launch.event_handlers.OnProcessExit(
                                         target_action=gazebo,
                                         on_exit=[launch.actions.EmitEvent(
                                             event=launch.events.Shutdown())],
                                      )),
                                   ])
