# Installation
package.xml
```bash
...
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>rclcpp</depend>
  <depend>ros_gz_bridge</depend>
  <depend>ros_gz_image</depend>
  <depend>ros_gz_sim</depend>
  <depend>sensor_msgs</depend>
  <depend>tf2</depend>
...
```
```bash
sudo apt install ros-humble-ros-gz-*
```
### ign_gazebo.launch.py
1.

```bash
pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
```

to
```bash
ros_gz_sim = get_package_share_directory('ros_gz_sim')
```
2. gz server
```bash
gzserver_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
) #args gz_sim instead of gz_args 
```
3.gz gui 
```bash
gzclient_cmd = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
    ),
    launch_arguments={'gz_args': '-g -v4 '}.items()
)
```
Finally, we need to set the environment variable GZ_SIM_RESOURCE_PATH so Gazebo can know where to find models. See the Finding resource document to learn more about this environment variable. This was not needed for gazebo_ros_pkgs because it used the <export> tag in package.xml to populate a similar environment variable for Gazebo (GAZEBO_MODEL_PATH).

```bash
set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                     'models'))

ld.add_action(set_env_vars_resources)
```

THEN launch


More than likely, this will fail because Gazebo could not find models referenced in the world SDFormat file. The next step is to fix that.

Note: Due to a bug in the GUI client, there might be a lingering ign gazebo -g or gz sim -g process after terminating the launch. You can kill it using pkill -f -9 'ign gazebo'

---------------

Edit world SDFormat file
The file we will be editing is turtlebot3_gazebo/worlds/empty_world.world. This file references the models sun and ground_plane. In Gazebo Classic, these models were either shipped with the simulator or downloaded from the gazebo model repository. The new Gazebo does not ship these models, instead we can use models from Fuel or add the models directly to the world file.

To use fuel models, replace the include tags for sun and ground_place with

```bash
<include>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane
  </uri>
</include>

<include>
  <uri>
    https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun
  </uri>
</include>
```
OLD world file turtlebot3_gazebo/worlds/empty_world.world
```bash
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <include>
      <uri>model://sun</uri>
    </include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

  </world>
</sdf>
```
NEW world file turtlebot3_gazebo/worlds/empty_world.world
```bash
<?xml version="1.0"?>
<sdf version="1.6">
  <world name="default">
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>
    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>
    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>
    <plugin
      filename="gz-sim-sensors-system"
      name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin
      filename="gz-sim-imu-system"
      name="gz::sim::systems::Imu">
    </plugin>

    <include>
      <uri>
        https://fuel.gazebosim.org/1.0/OpenRobotics/models/Ground Plane
      </uri>
    </include>

    <include>
      <uri>
        https://fuel.gazebosim.org/1.0/OpenRobotics/models/Sun
      </uri>
    </include>

    <scene>
      <shadows>false</shadows>
    </scene>

    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>0.319654 -0.235002 9.29441 0 1.5138 0.009599</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>

    <physics type="ode">
      <real_time_update_rate>1000.0</real_time_update_rate>
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1</real_time_factor>
      <ode>
        <solver>
          <type>quick</type>
          <iters>150</iters>
          <precon_iters>0</precon_iters>
          <sor>1.400000</sor>
          <use_dynamic_moi_rescaling>1</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0.00001</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>2000.000000</contact_max_correcting_vel>
          <contact_surface_layer>0.01000</contact_surface_layer>
        </constraints>
      </ode>
    </physics>

  </world>
</sdf>
```
### Spawn the model 
In this step, we will modify turtlebot3_gazebo/launch/spawn_turtlebot3.launch.py. Again, we need to change gazebo_ros to ros_gz_sim. We’ll also need to change spawn_entity.py to create, which is the node in ros_gz_sim that provides model spawning functionality. From the argument list, -entity needs to be replaced with -name. You can run ros2 run ros_gz_sim create --helpshort to see more options.

```bash
start_gazebo_ros_spawner_cmd = Node(
    package='ros_gz_sim',
    executable='create',
    arguments=[
        '-name', TURTLEBOT3_MODEL,
        '-file', urdf_path,
        '-x', x_pose,
        '-y', y_pose,
        '-z', '0.01'
    ],
    output='screen',
)
```
### modifying sdf files

old turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf

# old

```bash
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="turtlebot3_waffle">  
  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>

    <link name="base_footprint"/>

    <link name="base_link">

      <inertial>
        <pose>-0.064 0 0.048 0 0 0</pose>
        <inertia>
          <ixx>4.2111447e-02</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>4.2111447e-02</iyy>
          <iyz>0</iyz>
          <izz>7.5254874e-02</izz>
        </inertia>
        <mass>1.3729096e+00</mass>
      </inertial>

      <collision name="base_collision">
        <pose>-0.064 0 0.048 0 0 0</pose>
        <geometry>
          <box>
            <size>0.265 0.265 0.089</size>
          </box>
        </geometry>
      </collision>

      <visual name="base_visual">
        <pose>-0.064 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/waffle_base.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
    </link>

    <link name="imu_link">
      <sensor name="tb3_imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
        <plugin name="turtlebot3_imu" filename="libgazebo_ros_imu_sensor.so">
          <ros>
            <!-- <namespace>/tb3</namespace> -->
            <remapping>~/out:=imu</remapping>
          </ros>
        </plugin>
      </sensor>
    </link>

    <link name="base_scan">    
      <inertial>
        <pose>-0.052 0 0.111 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.114</mass>
      </inertial>

      <collision name="lidar_sensor_collision">
        <pose>-0.052 0 0.111 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.0508</radius>
            <length>0.055</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="lidar_sensor_visual">
        <pose>-0.064 0 0.121 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/lds.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>

      <sensor name="hls_lfcd_lds" type="ray">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <pose>-0.064 0 0.121 0 0 0</pose>
        <update_rate>5</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.120000</min>
            <max>3.5</max>
            <resolution>0.015000</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </ray>
        <plugin name="turtlebot3_laserscan" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <!-- <namespace>/tb3</namespace> -->
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>base_scan</frame_name>
        </plugin>
      </sensor>
    </link>

    <link name="wheel_left_link">

      <inertial>
        <pose>0.0 0.144 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>1.1175580e-05</ixx>
          <ixy>-4.2369783e-11</ixy>
          <ixz>-5.9381719e-09</ixz>
          <iyy>1.1192413e-05</iyy>
          <iyz>-1.4400107e-11</iyz>
          <izz>2.0712558e-05</izz>
        </inertia>
        <mass>0.1</mass>
      </inertial>

      <collision name="wheel_left_collision">
        <pose>0.0 0.144 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_left_visual">
        <pose>0.0 0.144 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/tire.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
    </link>

    <link name="wheel_right_link">

      <inertial>
        <pose>0.0 -0.144 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>1.1175580e-05</ixx>
          <ixy>-4.2369783e-11</ixy>
          <ixz>-5.9381719e-09</ixz>
          <iyy>1.1192413e-05</iyy>
          <iyz>-1.4400107e-11</iyz>
          <izz>2.0712558e-05</izz>
        </inertia>
        <mass>0.1</mass>
      </inertial>
    
      <collision name="wheel_right_collision">
        <pose>0.0 -0.144 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_right_visual">
        <pose>0.0 -0.144 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/tire.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
      </visual>
    </link>

    <link name='caster_back_right_link'>
      <pose>-0.177 -0.064 -0.004 -1.57 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.00001</iyy>
          <iyz>0.000</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.005000</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <link name='caster_back_left_link'>
      <pose>-0.177 0.064 -0.004 -1.57 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.00001</iyy>
          <iyz>0.000</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.005000</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <link name="camera_link"/>

    <link name="camera_rgb_frame">
      <inertial>
        <pose>0.069 -0.047 0.107 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.035</mass>
      </inertial>

      <pose>0.069 -0.047 0.107 0 0 0</pose>
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>30</update_rate>
        <camera name="intel_realsense_r200">
          <horizontal_fov>1.02974</horizontal_fov>
          <image>
            <width>1920</width>
            <height>1080</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>300</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <!-- Noise is sampled independently per pixel on each frame.
                  That pixel's noise value is added to each of its color
                  channels, which at that point lie in the range [0,1]. -->
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
          <plugin name="camera_driver" filename="libgazebo_ros_camera.so">
            <ros>
              <!-- <namespace>test_cam</namespace> -->
              <!-- <remapping>image_raw:=image_demo</remapping> -->
              <!-- <remapping>camera_info:=camera_info_demo</remapping> -->
            </ros>
            <!-- camera_name>omit so it defaults to sensor name</camera_name-->
            <!-- frame_name>omit so it defaults to link name</frameName-->
            <!-- <hack_baseline>0.07</hack_baseline> -->
          </plugin>
      </sensor>
    </link>    

    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent>
      <child>base_link</child>
      <pose>0.0 0.0 0.010 0 0 0</pose>
    </joint>

    <joint name="wheel_left_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_left_link</child>
      <pose>0.0 0.144 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="wheel_right_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_right_link</child>
      <pose>0.0 -0.144 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name='caster_back_right_joint' type='ball'>
      <parent>base_link</parent>
      <child>caster_back_right_link</child>
    </joint>

    <joint name='caster_back_left_joint' type='ball'>
      <parent>base_link</parent>
      <child>caster_back_left_link</child>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
      <pose>-0.032 0 0.068 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>    

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>base_scan</child>
      <pose>-0.064 0 0.121 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose>0.064 -0.065 0.094 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="camera_rgb_joint" type="fixed">
      <parent>camera_link</parent>
      <child>camera_rgb_frame</child>
      <pose>0.005 0.018 0.013 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <plugin name="turtlebot3_diff_drive" filename="libgazebo_ros_diff_drive.so">

      <ros>
        <!-- <namespace>/tb3</namespace> -->
      </ros>

      <update_rate>30</update_rate>

      <!-- wheels -->
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>0.287</wheel_separation>
      <wheel_diameter>0.066</wheel_diameter>

      <!-- limits -->
      <max_wheel_torque>20</max_wheel_torque>
      <max_wheel_acceleration>1.0</max_wheel_acceleration>

      <command_topic>cmd_vel</command_topic>

      <!-- output -->
      <publish_odom>true</publish_odom>
      <publish_odom_tf>true</publish_odom_tf>
      <publish_wheel_tf>false</publish_wheel_tf>

      <odometry_topic>odom</odometry_topic>
      <odometry_frame>odom</odometry_frame>
      <robot_base_frame>base_footprint</robot_base_frame>

    </plugin>

    <plugin name="turtlebot3_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <!-- <namespace>/tb3</namespace> -->
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>wheel_left_joint</joint_name>
      <joint_name>wheel_right_joint</joint_name>
    </plugin>    

  </model>
</sdf>
```
# new 
```bash
<?xml version="1.0" ?>
<sdf version="1.5">
  <model name="turtlebot3_waffle">  
  <pose>0.0 0.0 0.0 0.0 0.0 0.0</pose>

    <link name="base_footprint"/>

    <link name="base_link">

      <inertial>
        <pose>-0.064 0 0.048 0 0 0</pose>
        <inertia>
          <ixx>4.2111447e-02</ixx>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyy>4.2111447e-02</iyy>
          <iyz>0</iyz>
          <izz>7.5254874e-02</izz>
        </inertia>
        <mass>1.3729096e+00</mass>
      </inertial>

      <collision name="base_collision">
        <pose>-0.064 0 0.048 0 0 0</pose>
        <geometry>
          <box>
            <size>0.265 0.265 0.089</size>
          </box>
        </geometry>
      </collision>

      <visual name="base_visual">
        <pose>-0.064 0 0 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/waffle_base.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>

    <link name="imu_link">
      <sensor name="tb3_imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
        <topic>imu</topic>
        <imu>
          <angular_velocity>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>2e-4</stddev>
              </noise>
            </z>
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </x>
            <y>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </y>
            <z>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>1.7e-2</stddev>
              </noise>
            </z>
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <link name="base_scan">    
      <inertial>
        <pose>-0.052 0 0.111 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.114</mass>
      </inertial>

      <collision name="lidar_sensor_collision">
        <pose>-0.052 0 0.111 0 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.0508</radius>
            <length>0.055</length>
          </cylinder>
        </geometry>
      </collision>

      <visual name="lidar_sensor_visual">
        <pose>-0.064 0 0.121 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/lds.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>

      <sensor name="hls_lfcd_lds" type="gpu_lidar">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <pose>-0.064 0 0.121 0 0 0</pose>
        <update_rate>5</update_rate>
        <topic>scan</topic>
        <gz_frame_id>base_scan</gz_frame_id>
        <lidar>
          <scan>
            <horizontal>
              <samples>360</samples>
              <resolution>1.000000</resolution>
              <min_angle>0.000000</min_angle>
              <max_angle>6.280000</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.120000</min>
            <max>3.5</max>
            <resolution>0.015000</resolution>
          </range>
          <noise>
            <type>gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </lidar>
      </sensor>
    </link>

    <link name="wheel_left_link">

      <inertial>
        <pose>0.0 0.144 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>1.1175580e-05</ixx>
          <ixy>-4.2369783e-11</ixy>
          <ixz>-5.9381719e-09</ixz>
          <iyy>1.1192413e-05</iyy>
          <iyz>-1.4400107e-11</iyz>
          <izz>2.0712558e-05</izz>
        </inertia>
        <mass>0.1</mass>
      </inertial>

      <collision name="wheel_left_collision">
        <pose>0.0 0.144 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_left_visual">
        <pose>0.0 0.144 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/tire.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>

    <link name="wheel_right_link">

      <inertial>
        <pose>0.0 -0.144 0.023 -1.57 0 0</pose>
        <inertia>
          <ixx>1.1175580e-05</ixx>
          <ixy>-4.2369783e-11</ixy>
          <ixz>-5.9381719e-09</ixz>
          <iyy>1.1192413e-05</iyy>
          <iyz>-1.4400107e-11</iyz>
          <izz>2.0712558e-05</izz>
        </inertia>
        <mass>0.1</mass>
      </inertial>
    
      <collision name="wheel_right_collision">
        <pose>0.0 -0.144 0.023 -1.57 0 0</pose>
        <geometry>
          <cylinder>
            <radius>0.033</radius>
            <length>0.018</length>
          </cylinder>
        </geometry>
        <surface>
          <!-- This friction pamareter don't contain reliable data!! -->
          <friction>
            <ode>
              <mu>100000.0</mu>
              <mu2>100000.0</mu2>
              <fdir1>0 0 0</fdir1>
              <slip1>0.0</slip1>
              <slip2>0.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="wheel_right_visual">
        <pose>0.0 -0.144 0.023 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://turtlebot3_common/meshes/tire.dae</uri>
            <scale>0.001 0.001 0.001</scale>
          </mesh>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
          <emissive>0 0 0 1</emissive>
        </material>
      </visual>
    </link>

    <link name='caster_back_right_link'>
      <pose>-0.177 -0.064 -0.004 -1.57 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.00001</iyy>
          <iyz>0.000</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.005000</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <link name='caster_back_left_link'>
      <pose>-0.177 0.064 -0.004 -1.57 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.00001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.00001</iyy>
          <iyz>0.000</iyz>
          <izz>0.00001</izz>
        </inertia>
      </inertial>
      <collision name='collision'>
        <geometry>
          <sphere>
            <radius>0.005000</radius>
          </sphere>
        </geometry>
        <surface>
          <contact>
            <ode>
              <soft_cfm>0</soft_cfm>
              <soft_erp>0.2</soft_erp>
              <kp>1e+5</kp>
              <kd>1</kd>
              <max_vel>0.01</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>
    </link>

    <link name="camera_link"/>

    <link name="camera_rgb_frame">
      <inertial>
        <pose>0.069 -0.047 0.107 0 0 0</pose>
        <inertia>
          <ixx>0.001</ixx>
          <ixy>0.000</ixy>
          <ixz>0.000</ixz>
          <iyy>0.001</iyy>
          <iyz>0.000</iyz>
          <izz>0.001</izz>
        </inertia>
        <mass>0.035</mass>
      </inertial>

      <pose>0.069 -0.047 0.107 0 0 0</pose>
      <sensor name="camera" type="camera">
        <always_on>true</always_on>
        <visualize>true</visualize>
        <update_rate>30</update_rate>
        <topic>camera/image_raw</topic>
        <gz_frame_id>camera_rgb_frame</gz_frame_id>
        <camera name="intel_realsense_r200">
          <camera_info_topic>camera/camera_info</camera_info_topic>
          <horizontal_fov>1.02974</horizontal_fov>
          <image>
            <width>1920</width>
            <height>1080</height>
            <format>R8G8B8</format>
          </image>
          <clip>
            <near>0.02</near>
            <far>300</far>
          </clip>
          <noise>
            <type>gaussian</type>
            <!-- Noise is sampled independently per pixel on each frame.
                  That pixel's noise value is added to each of its color
                  channels, which at that point lie in the range [0,1]. -->
            <mean>0.0</mean>
            <stddev>0.007</stddev>
          </noise>
        </camera>
      </sensor>
    </link>    

    <joint name="base_joint" type="fixed">
      <parent>base_footprint</parent>
      <child>base_link</child>
      <pose>0.0 0.0 0.010 0 0 0</pose>
    </joint>

    <joint name="wheel_left_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_left_link</child>
      <pose>0.0 0.144 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <effort>20</effort>
        </limit>
      </axis>
    </joint>

    <joint name="wheel_right_joint" type="revolute">
      <parent>base_link</parent>
      <child>wheel_right_link</child>
      <pose>0.0 -0.144 0.023 -1.57 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
        <limit>
          <effort>20</effort>
        </limit>
      </axis>
    </joint>

    <joint name='caster_back_right_joint' type='ball'>
      <parent>base_link</parent>
      <child>caster_back_right_link</child>
    </joint>

    <joint name='caster_back_left_joint' type='ball'>
      <parent>base_link</parent>
      <child>caster_back_left_link</child>
    </joint>

    <joint name="imu_joint" type="fixed">
      <parent>base_link</parent>
      <child>imu_link</child>
      <pose>-0.032 0 0.068 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>    

    <joint name="lidar_joint" type="fixed">
      <parent>base_link</parent>
      <child>base_scan</child>
      <pose>-0.064 0 0.121 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>camera_link</child>
      <pose>0.064 -0.065 0.094 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <joint name="camera_rgb_joint" type="fixed">
      <parent>camera_link</parent>
      <child>camera_rgb_frame</child>
      <pose>0.005 0.018 0.013 0 0 0</pose>
      <axis>
        <xyz>0 0 1</xyz>
      </axis>
    </joint>

    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <!-- wheels -->
      <left_joint>wheel_left_joint</left_joint>
      <right_joint>wheel_right_joint</right_joint>

      <!-- kinematics -->
      <wheel_separation>0.287</wheel_separation>
      <wheel_radius>0.033</wheel_radius>

      <!-- limits -->
      <max_linear_acceleration>1.0</max_linear_acceleration>

      <topic>cmd_vel</topic>


      <odom_topic>odom</odom_topic>
      <frame_id>odom</frame_id>
      <tf_topic>/tf</tf_topic>
      <odom_publisher_frequency>30</odom_publisher_frequency>
      <child_frame_id>base_footprint</child_frame_id>

    </plugin>

    <plugin filename="gz-sim-joint-state-publisher-system"
      name="gz::sim::systems::JointStatePublisher">
      <topic>joint_states</topic>
      <!-- <update_rate>30</update_rate> -->
      <joint_name>wheel_left_joint</joint_name>
      <joint_name>wheel_right_joint</joint_name>
    </plugin>    

  </model>
</sdf>
```

#### why

If you uncomment ld.add_action(spawn_turtlebot_cmd) in empty_world.launch.py and run the launch file, you’ll notice errors related to unrecognized plugins. These are coming from the model SDFormat file.
## libgazebo_ros_imu_sensor.so


old plugin in sdf
```bash
 link name="imu_link">
      <sensor name="tb3_imu" type="imu">
        <always_on>true</always_on>
        <update_rate>200</update_rate>
```
new 
```bash


<link name="imu_link">
  <sensor name="tb3_imu" type="imu">
    <always_on>true</always_on>
    <update_rate>200</update_rate>
    <topic>imu</topic>
    <imu>
      ... <!-- all the content of <imu> -->
    </imu>
  </sensor>
</link>
```

## libgazebo_ros_camera.so
old

```bash
<plugin name="turtlebot3_laserscan" filename="libgazebo_ros_ray_sensor.so">
          <ros>
            <!-- <namespace>/tb3</namespace> -->
            <remapping>~/out:=scan</remapping>
          </ros>
          <output_type>sensor_msgs/LaserScan</output_type>
          <frame_name>base_scan</frame_name>
        </plugin>
```

new 

```bash
<sensor name="camera" type="camera">
  <always_on>true</always_on>
  <visualize>true</visualize>
  <update_rate>30</update_rate>
  <topic>camera/image_raw</topic>
  <gz_frame_id>camera_rgb_frame</gz_frame_id>
  <camera name="intel_realsense_r200">
    <camera_info_topic>camera/camera_info</camera_info_topic>
    ... <!-- all the content of <camera> from the original -->
  </camera>
</sensor>
```

# old launch file
```bash
#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
```
# new launch file 
```bash
#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Joep Tool

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )
    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'models'))

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v4 ', world], 'on_exit_shutdown': 'true'}.items()
    )
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v4 '}.items()
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
        }.items()
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(set_env_vars_resources)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
```


# final 

```bash
from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='tortoisebot_gazebo').find('tortoisebot_gazebo')
    world_path = os.path.join(pkg_share, 'worlds/room2.sdf')
    use_sim_time = LaunchConfiguration('use_sim_time')

    ign_gazebo = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory('ros_ign_gazebo'), 'launch', 'ign_gazebo.launch.py'),
        launch_arguments={'ign_args': '-r -v 4 empty.sdf'}.items()
    )

    return launch.LaunchDescription([
        # launch.actions.ExecuteProcess(cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_path], output='screen'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='False', description='Flag to enable use_sim_time'),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     arguments=['-entity', 'tortoisebot', '-topic', 'robot_description'],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen',
        # )
        ign_gazebo
    ])

```
