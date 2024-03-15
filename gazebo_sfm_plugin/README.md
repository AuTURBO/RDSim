# gazebo_sfm_plugin
A plugin for simulation of human pedestrians in ROS2 and Gazebo.

**Tested in ROS2 Galactic and Gazebo 11** 

The persons are affected by the obstacles and other persons using the [Social Force Model](https://github.com/robotics-upo/lightsfm)


![](https://github.com/robotics-upo/gazebo_sfm_plugin/blob/master/media/images/capture3.jpg)


## Plugin configuration

The plugin can be applied to each Gazebo Actor indicated in the Gazebo world file.

An example snippet is shown next:

```html
<actor name="actor1">
	<pose>-1 2 1.25 0 0 0</pose>
	<skin>
		<filename>walk.dae</filename>
		<scale>1.0</scale>
	</skin>
	<animation name="walking">
		<filename>walk.dae</filename>
		<scale>1.000000</scale>
		<interpolate_x>true</interpolate_x>
	</animation>
	<!-- plugin definition -->
	<plugin name="actor1_plugin" filename="libPedestrianSFMPlugin.so">
		<velocity>0.9</velocity>
		<radius>0.4</radius>
		<animation_factor>5.1</animation_factor>
		<people_distance>6.0</people_distance>
		<!-- weights -->
		<goal_weight>2.0</goal_weight>
		<obstacle_weight>80.0</obstacle_weight>
		<social_weight>15</social_weight>
		<group_gaze_weight>3.0</group_gaze_weight>
		<group_coh_weight>2.0</group_coh_weight>
		<group_rep_weight>1.0</group_rep_weight>
		<ignore_obstacles>
			<model>cafe</model>
			<model>ground_plane</model>
		</ignore_obstacles>
		<trajectory>
			<cyclic>true</cyclic>
			<waypoint>-1 2 1.25</waypoint>
			<waypoint>-1 -8 1.25</waypoint>
		</trajectory>
	</plugin>
</actor>
```
The parameters that can be configured for each pedestrian are:

### General params

*  ```<velocity>```. Maximum velocity (*m/s*) of the pedestrian.
*  ```<radius>```. Approximate radius of the pedestrian's body (m).
*  ```<animation_factor>```. Factor employed to coordinate the animation with the walking velocity.
* ```<people_distance>```.  Maximum detection distance of the surrounding pedestrians.

### SFM Weights

*  The weight factors that modify the navigation behavior. See the [Social Force Model](https://github.com/robotics-upo/lightsfm) for further information.

### Obstacle params

* ```<ignore_obstacles>```.  All the models that must be ignored as obstacles, must be indicated here. The other actors in the world are included automatically.

### Trajectory params

* ```<trajectory>```. The list of waypoints that the actor must reach must be indicated here. 

	- ```<waypoint>```. Each waypoint must be indicated by its coordinates X, Y, Z in the world frame.
	- ```<cyclic>```. If true, the actor will start the waypoint sequence when the last waypoint is reached.

## Dependencies

* Yo must download and install the Social Force Model library, lightsfm https://github.com/robotics-upo/lightsfm

```sh
git clone https://github.com/robotics-upo/lightsfm
cd lightsfm
make
sudo make install
```

## Compilation

* This is a ROS2 package so it must be placed inside a ROS2 workspace and compiled through the regular colcon compiler. 
```sh
colcon build --packages-select gazebo_sfm_plugin
source ~/ros2_ws/install/setup.bash
```
* Setting GAZEBO_RESOURCE_PATH
```sh
echo "export GAZEBO_RESOURCE_PATH='$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11'" >> ~/.bashrc
source ~/.bashrc
```

## Example

An example Gazebo world can be launched through:
```sh
ros2 launch gazebo_sfm_plugin cafe_ros2.launch.py
```

