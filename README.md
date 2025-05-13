# ora25_ws


The ORA software for 2025.

## To work on the code

### Clone the repo
```
mkdir ora25_ws && cd ora25_ws
git clone https://github.com/oaklandrobotics/ros_ora25 src
```

### Write some code!

This can be done in your IDE of choice, such as VS Code.

## Packages
### Generic Packages
- `odrive_base`
  - Base ODrive package, includes helper packages
- `odrive_can`
  - Package used for communication with the ODrives using the CANBus protocol. This provides some functionality, but is not being used atm.
- `odrive_ros2_control`
  - Package for ros2 control integration with the ODrives. This is what we will be using for our primary control.
- `sllidar_ros2`
  - Package for the RPLidar we are using in our project
- `zed_robot_integration`
### ORA Packages
- `ora_description`
  - Contains the description of the robot, including its frame, sensors, control, etc.
- `ora_navigation`
  - Contains the NAV2 functionality
- `ora_slam_toolbox`
  - Contains custom slam_toolbox configuration
- `ora_teleop`
  - Contains the configuration for teleop through a controller
    - `LB`
      - Normal deadman switch
    - `RB`
      - Turbo deadman switch
    - `LY`
      - FW/BW robot control
    - `RX`
      - Left/Right robot control
    - `A`
      - Restart ODrive control
    - `B`
      - EStop

## Running the code
### Building the workspace

Start by navigating to the `ora25_ws` folder.

Use the following commands to build and source your workspace
```
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Now you are ready to run some of the packages!

Try out ~
```
ros2 launch ora_description launch_sim.launch.py
ros2 launch ora_teleop joystick.launch.py \ ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 launch ora_navigation twist_mux.launch.py
```

~ to get started with a simulation that can be controlled through a connected controller or the `teleop_twist_keyboard`
