# HitchOpen-LGSVL

This repository contains the autonomous driving system developed for the HitchOpen project. The system is divided into five main components: perception, localization, planning, control, and simulation.

## Getting Started

### System Requirements
- **Operating System**: Ubuntu 22.04 in native OS, virtual machine, or Windows Subsystem for Linux (WSL). Ubuntu 22.04 is what the race car runs on and it is strongly recommended you develop in native Ubuntu 22.04 Linux to avoid issues.
- **ROS Distribution**: [ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html)
- Notes: ARM64 architecture is untested.

### Hardware Recommandation
- **CPU**: 12th Gen Intel® Core™ i9-12900HK × 20 or above
- **GPU**: NVIDIA GeForce 3080 or above (for simulation)
- **Memory**: 64.0 GiB or above (for simulation)

### Dependencies
- **common.iron.repos**: Provides basic message types and modules for the stack. Most of them are modeified version from [The Autoware Foundation](https://github.com/autowarefoundation). [Autoware.universe](https://github.com/autowarefoundation/autoware.universe) provides many useful and insightful algorithms for autonomous driving stacks. 
- **svl.iron.repos**: Provides bridge to simulation

### Installation
We will assume you have ROS2 Iron installed and sourced in all these terminals.

1. Fork `HitchOpen-LGSVL` and name your fork as desired. Then clone your fork and navigate to the repository root:
   ```
   git clone YOUR_REPO_GITHUB_PATH
   cd YOUR_REPO_NAME
   ```

2. Install `python3-vcstool`:
   ```bash
   sudo apt update
   sudo apt install python3-vcstool
   ```

3. Import dependencies:
   ```bash
   make vcs-import VCS_FILE=common.iron.repos
   ```

4. Install dependencies. In general if you run a VCS step post this then you need to rerun the `make rosdep-install` command:
   ```bash
   make rosdep-install-eol
   pip3 install environs
   ```

5. Build up the stack:
   ```bash
   make svl 
   ```

## Perception

The perception system is responsible for understanding the environment around the vehicle through various sensors and processing algorithms.

#### Dependencies
- `Autoware Universe`: https://github.com/autowarefoundation/autoware.universe


## Localization

The localization system determines the precise position and orientation of the vehicle in the environment.

#### Dependencies
- `Robot Localization`: https://github.com/cra-ros-pkg/robot_localization

#### Key Algorithms:
- `odometry_state_estimator`: Dynamically calibrates wheels speed from wheel reports and GPS, and outputs smoothed linear and angular velocity as odometry. 
- `vehicle_kinematic_state`: Merges odometry topics from different sources with local and global EKF, and outputs vehicle kinematic state.


## Planning

The planning system is responsible for generating safe and optimal global trajectories for autonomous navigation.

The offline created trajactories are stored in `src/common/race_metadara/ttls`, and the definition of each column are as follows(`src/common/target_trajectory_line/include/ttl.hpp`):

```
struct TtlHeader
{
  std::string ttl_name; # File name
  TtlIndex number; # Number of the trajectory
  uint32_t loop; # Number of points
  double total_distance; # Meters
  GpsPosition origin; # GPS Coordinate
};

enum TtlColumn
{
  X = 0, #ENU Coordinate
  Y = 1,
  Z = 2, 
  TARGET_YAW = 3, # Rad
  TARGET_SPEED = 4, # Mile per hour
};
```


## Control

The control system is responsible for executing the planned trajectory by generating appropriate steering and velocity commands. 

#### Key Algorithms:
- `Local Path Planner`: Generates a smooth, feasible trajectory considering vehicle dynamics and constraints.
- `Pure Pursuit Controller`: Provides robust path tracking by computing a look-ahead point on the path ahead of the vehicle, calculating steering commands to guide the vehicle toward this point and dynamically adjusting look-ahead distance based on vehicle speed.


## Simulation

We maintain a private fork of SVL simulator which adds richer vehicle dynamics to the vehicle. Combined with accurate sensor configuration, SVL was the ultimate solution for integration testing. 

#### Chassis
   - Overall Length: 192in/4876mm
   - Overall Width: 76in/1930mm
   - Overall Height: 45.5in/1156.5mm
   - Wheelbase: 117in/2972mm
   - Curb Weight: 1680lbs/748.43kg

#### Installation Steps
1. Download the release from [here](https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/tianlun_zhang_intelligentracing_com/EVBmsMrNlrVCob8QErwZ7jEBM3x6QQZZTEedvdhRlpJm-w?e=J1WU67) and unzip it.
2. Download the `wise.zip` file from [here](https://intelligentracingcom-my.sharepoint.com/:u:/g/personal/tianlun_zhang_intelligentracing_com/ES8QdS9l-pBKgFhAtyaUQuQBil8Q7Qc9UqUZAgeWjPncOg?e=2RKUGa) and unzip it in the same folder as the simulator build
3. Set up [OSSDC API](https://github.com/lgsvl/PythonAPI?tab=readme-ov-file):
   ```
   git clone git@github.com:lgsvl/PythonAPI.git
   cd PythonAPI
   python3 -m pip install -r requirements.txt --user .
   ```
4. Install the correct NVIDIA driver: https://documentation.ubuntu.com/server/how-to/graphics/install-nvidia-drivers/index.html.

#### Expected Folder Structure
After extraction, you should have a folder structure like this:
```
OSSDC-SIM-ART-Linux # (if you downloaded the Linux version)
├── data
├── wise
├── OSSDC-SIM
├── start_sim_local.sh
├── start_sim_cloud.sh
├── config.yaml
└── ...
```

#### Running the Simulator

##### Terminal Types
| Terminal | Type | Description |
|----------|------|-------------|
| Pilot Terminal | WSL terminal / Linux terminal | A terminal open in the `HitchOpen-LGSVL` folder and sourced. This can be done through `source ./source_all.sh`. If it does not work, please manually source ROS and your workspace properly. |
| OSSDC Sim Terminal | Windows cmd / WSL terminal / Linux Terminal | A terminal open in the simulator folder (`OSSDC-ART-Linux`) |

##### Manual Setup
Please source all the terminals properly. Please make sure your ports are not maunally assigned for other purposes. 

1. Start the wise server (in OSSDC Sim Terminal):
   ```bash
   cd wise
   python3 -m http.server 9090
   ```
   Note: To change the port from 9090, edit `OSSDC-SIM-ART-Windows/start_sim_local.bat` or `OSSDC-SIM-ART-Linux/start_sim_local.sh`

2. Start the simulator (in OSSDC Sim Terminal):
   ```bash
   cd OSSDC-SIM-ART-Linux
   ./start_sim_local.sh
   ```

3. Start the ROS2 bridge (in Pilot Terminal):
   ```bash
   lgsvl_bridge --port 9091
   ```

4. Start the Python API (in Pilot Terminal):
   ```bash
   python3 src/launch/svl_launch/scripts/launch_ossdc.py --env svl.env
   ```
   Note: Map, sensors, and other configurations can be modified in `race.env`. Spawn location and initial speed can be modified in `uuids.json`


5. Launch urdf if needed(in Pilot Terminal):
   ```bash
   ros2 launch autonomy_launch misc.launch.py
   ```


6. You can directly control the car by publishing topic `/lgsvl/control`. The definition can be found here: https://github.com/lgsvl/lgsvl_msgs/blob/master/msg/VehicleControlData.msg
   ```
   std_msgs/Header header

   float32 acceleration_pct  # 0 to 1
   float32 braking_pct  # 0 to 1
   float32 target_wheel_angle  # radians
   float32 target_wheel_angular_rate  # radians / second
   uint8 target_gear
   ```

7. Alternatively, keynoard_controller is provided to explore the maps and collect waypoints. You can use `WASD` to maunally control the vehicle in the simulator (in Pilot Terminal):
   ```
   ros2 run keyboard_controller keyboard_control
   ```

8. You can also run our sample stack (in Pilot Terminal):
   ```bash
   ros2 launch simple_racing simple_racing.launch.py params_file:=src/launch/simple_racing/params/simple_racing.yml
   ```

#### Simulation Competition

The simulation environment provides a comprehensive race competition system with integrated timing and flag control. This allows for realistic race scenario testing and evaluation.

1. To ensure fairness in the simulation competition, we ask all the teams NOT to modify the following modules:
   - All packages under `src/external/autoware`
   - `svl_launch`
   - `competition_timer`

2. Competition timer (Please do not modify anything in competition_timer for your final submission)
   - Keeps track of the laps number and clocks the time. 
   - Configuration options:
      - Map is set in `race.env`
      - `results_dir`: Directory to save competition results
      - `target_laps`: Number of laps to complete (default: 10)
      - `vehicle_flag`: Initial flag (default: 'red')
   - You can start the competition timer by:
      ```bash
      ros2 launch competition_timer competition_timer.launch.py use_sim_time:=true
      ```
      
3. Flags
   - The competition timer publishes VehicleFlag on the `/vehicle_flag` topic
   - Flags in the competition:
     - GREEN: Competition is active, vehicles can go
     - RED: Vehicle must stop
     - BLACK: Competition ends
   - The competition is started by manually setting the flag to green:
      ```bash
      ros2 param set /competition_timer_node vehicle_flag green
      ```
   - Vehicles MUST subscribe to `/vehicle_flag` topic and respond to flag states. If no flag is received, vehicle must assume RED flag.
   - Black flag is published upon the completion of the target laps or early termination. You do not need to respond to it in the simulation now, but please try to make sure your vehicles can stop safely after finishing the competition.

4. Competition States
   - Inactive: Simulation Competition is launched with deault RED flag. Vechiles are not allowed to move.
   - Start: Competition begins with GREEN flag. Timer starts. 
   - Pause: Red flag might be published during the competition due to safety reasons and abnormal vehicle behavior. Timer does not pasue.
   - End: Vehicles have finished the target_laps or BLACK flag is published early to stop the competition.

5. Submission
   - Your submission MUST be a fork of the official competition repository.
   - When submitting your competition result, please ensure your team updates the instructions clearly in your fork’s GitHub README. Otherwise, we will execute and evaluate your submission with the default setting provided in the official repository. 
