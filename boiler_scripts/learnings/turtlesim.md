# SERVICES 

- `/clear` : clears the background of trail lines
- `/kill` : to kill a turtle
- `/reset` : resets the position of the turtle
- `/spawn` : to spawn a turtle
- `/turtle1/set_pen` : sets the color and thickness of the trail line.
- `/turtle1/teleport_absolute` : move the turtle instantly

## Info about Service

```bash
ros2 service type <service_name>
ros2 service call -r 0.5 /spawn turtlesim/srv/Spawn "{x: 5, y: 5, theta: 0}"
```

### Structure

```bash
ros2 service call <service_name> <service_type> <arguments>
```

### 1. Simple services like `/clear`, `/kill`, `/reset`, `/spawn` (structure)

- `/clear`:
```bash
ros2 service call /clear std_srvs/srv/Empty
```

- `/kill`: 
```bash
ros2 service call /kill turtlesim/srv/Kill "{name: 'turtle1'}"
```

- `/reset`: 
*(No command provided)*

- `/spawn`: 
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 5.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```

### 2. `/turtle1/set_pen`, `/turtle1/teleport_absolute`, `/turtle1/teleport_relative` (structure)

- `/set_pen`: 
```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 2, 'off': 0}"
```

- `/turtle1/teleport_absolute`: 
```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.0, y: 5.0, theta: 0.0}"
```

- `/turtle1/teleport_relative`: 
*(No command provided)*

### 3. For the turtlesim parameter services like `/turtlesim/describe_parameters`, `/turtlesim/get_parameters`

- `get_parameters`: 
```bash
ros2 service call /turtlesim/get_parameters rcl_interfaces/srv/GetParameters "{names: ['background_r', 'background_g', 'background_b']}"
```

- `/turtlesim/describe_parameters`: 
*(No command provided)*

# ACTIONS

- To get action information:
```bash
ros2 action info action_name
```

- To send a goal:
```bash
ros2 action send_goal <action_name> <action type> <info>
ros2 action send_goal /turtle1/rotate_absolute turtlesim/action/RotateAbsolute "{'theta: 1.57'}" --feedback
```

# MOVEMENT

### Line:
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### Circle:
```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8"
```

# REMAPPING TOPICS FOR MULTIPLE TURTLES

When you spawn another turtle, you need a second teleop node to control `turtle2`. Running the same command as before will still control `turtle1`. To change this behavior, remap the `cmd_vel` topic.

```bash
ros2 run turtlesim turtle_teleop_key --ros-args --remap turtle1/cmd_vel:=turtle2/cmd_vel
```

To control `turtle2`:
```bash
ros2 topic pub /turtle2/cmd_vel geometry_msgs/msg/Twist "linear:
  x: -2.0
  y: -2.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.0"
```
