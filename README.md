# 🤖 `ibt_ros2_autodocking`

---

## 🎯 Overview

`ibt_ros2_autodocking` is a **ROS 2** package designed to automate the **docking and undocking** procedures for a mobile robot. It implements a robust and precise solution that combines high-level navigation provided by **Nav2** with fine alignment based on vision and **ArUco marker** detection. The package includes both an **Action Server** to manage the multi-phase docking logic and an **Action Client** to request and monitor these operations.

---

## ✨ Key Features

* **Multi-Phase Docking:** A docking procedure structured in several phases to ensure precision and reliability.
* **Nav2 Integration:** Leverages Nav2's navigation capabilities for coarse approach to the docking station.
* **ArUco Precision Alignment:** Utilizes ArUco markers and camera vision for millimeter-level corrections in position and orientation.
* **Charge Verification:** Includes a battery status verification phase to confirm that the robot is actually charging after docking.
* **Undocking Procedure:** Supports a controlled backward movement to move away from the dock.
* **Configurable Parameters:** Numerous parameters are exposed for easy tuning and adaptation to different robots or environments.
* **ROS 2 Action Interface:** Provides a standardized interface (`ibt_ros2_interfaces/action/Docking`) for interaction with other system nodes.

---

## 📂 Package Structure

The `ibt_ros2_autodocking` package is organized as follows:

```

ibt\_ros2\_autodocking/
│  
├── ibt\_ros2\_autodocking/       \# Folder containing the package's Python source code
│   ├── **init**.py             \# File indicating that the directory is a Python module
│   ├── docking\_server\_node.py  \# The Action Server node for docking/undocking logic
│   └── docking\_client\_node.py  \# The Action Client node for interacting with the server
├── resource/                   \# Folder for resource files (required by ament\_python)
│   └── ibt\_ros2\_autodocking    \# Specific resource file for the package
├── .gitignore                  \# File to specify items to ignore in version control (Git)
├── package.xml                 \# The ROS 2 package manifest (metadata and dependencies)
├── setup.py                    \# Python build and installation script (used by setuptools/colcon)
├── setup.cfg                   \# Configuration file for setuptools
└── README.md                   \# This documentation file

````

---

## 🚀 Requirements and Dependencies

This package requires a **ROS 2 Humble** (or newer) environment and the following dependencies. Make sure they are installed on your ROS system.

### ROS 2 Packages

* [`rclpy`](https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html#rclpy-python-client-library)
* [`rclpy_action`](https://docs.ros.org/en/humble/Concepts/About-Client-Libraries.html#action-clients-and-servers)
* [`sensor_msgs`](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Interfaces.html#common-interfaces-and-messages)
* [`geometry_msgs`](https://docs.ros.org/en/humble/Concepts/About-ROS-2-Interfaces.html#common-interfaces-and-messages)
* [`nav2_msgs`](https://navigation.ros.org/package_references/nav2_msgs.html)
* [`cv_bridge`](https://wiki.ros.org/cv_bridge)
* [`tf2_ros`](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Python.html)
* [`tf2_geometry_msgs`](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Python.html)
* [`ibt_ros2_interfaces`](https://github.com/YourOrg/ibt_ros2_interfaces) (your custom action definitions package)

### Python Libraries (installable via `pip`)

* `setuptools`
* `numpy`
* `opencv-python` (and potentially `opencv-contrib-python` for some ArUco modules)
* `tf-transformations`

#### Dependency Installation

1.  **Update `rosdep`** (if necessary):
    ```bash
    sudo rosdep init
    rosdep update
    ```
2.  **Install ROS dependencies** (from your ROS 2 workspace):
    ```bash
    rosdep install -i --from-path src --rosdistro humble -y
    ```
3.  **Install Python dependencies** (you can create a `requirements.txt` or install them manually):
    ```bash
    # Option 1: Create a requirements.txt file in the package root with the Python libraries
    # Example requirements.txt:
    # numpy
    # opencv-python
    # opencv-contrib-python
    # tf-transformations
    #
    # Then install:
    # pip install -r src/ibt_ros2_autodocking/requirements.txt
    
    # Option 2: Manual installation (recommended for quick development)
    pip install numpy opencv-python opencv-contrib-python tf-transformations
    ```

**Note on Vision (OpenCV/CvBridge):** Depending on your ROS 2 distribution and operating system, you might need to install specific versions of `opencv-python` or the `python3-opencv` package via your system's package manager (e.g., `sudo apt install ros-humble-opencv-ros-py`). Ensure your OpenCV installation supports `aruco` modules.

---

## 🛠️ Build the Package

To compile and install the package in your ROS 2 workspace:

1.  **Navigate to your ROS 2 workspace:**
    ```bash
    cd ~/ros2_ws/
    ```

2.  **Clone the necessary repositories** (if you haven't already. Ensure `ibt_ros2_interfaces` is in the same workspace):
    ```bash
    # git clone <URL_of_your_autodocking_repo> src/ibt_ros2_autodocking
    # git clone <URL_of_your_interfaces_repo> src/ibt_ros2_interfaces
    ```

3.  **Build the workspace** with `colcon`:
    ```bash
    colcon build --symlink-install --packages-select ibt_ros2_autodocking ibt_ros2_interfaces
    ```
    * Use `--symlink-install` for development mode installations, which is convenient for quick changes to Python code.
    * `--packages-select ibt_ros2_autodocking ibt_ros2_interfaces` compiles only the specified packages, speeding up the process.

4.  **Source the ROS 2 environment** to make the new packages available:
    ```bash
    source install/setup.bash
    ```

---

## 🚀 Usage

### Sourcing the Environment

Before running any nodes, make sure you have correctly sourced your ROS 2 environment in each new terminal:

```bash
source /opt/ros/humble/setup.bash # Replace 'humble' with your ROS 2 distribution
source ~/ros2_ws/install/setup.bash
````

### Running the Docking Server

Start the docking action server in a separate terminal. It's recommended to load parameters from docking_server_node file for more control:

```bash
# Running with default parameters (for quick tests)
ros2 run ibt_ros2_autodocking docking_server_node


### Using the Docking Client

Once the server is running, you can send commands using the client from another terminal:

  * **Start the docking procedure:**
    ```bash
    ros2 run ibt_ros2_autodocking docking_client_node dock
    ```
  * **Start the undocking procedure:**
    ```bash
    ros2 run ibt_ros2_autodocking docking_client_node undock
    ```
  * **Monitor real-time battery status:**
    ```bash
    ros2 run ibt_ros2_autodocking docking_client_node percentage
    ```

-----

## ⚙️ Node Parameters

The `DockingActionServer` exposes numerous parameters that can be configured via a ROS 2 launch YAML file to adapt to your specific setup, allowing fine-tuning of the navigation and alignment phases.

| Parameter                      | Type   | Default | Description                                                                    |
| :----------------------------- | :----- | :------ | :----------------------------------------------------------------------------- |
| `phase0_goal_x`                | double | 0.13    | X coordinate of the initial Nav2 goal in Phase 0.                              |
| `phase0_goal_y`                | double | 0.87    | Y coordinate of the initial Nav2 goal in Phase 0.                              |
| `phase0_kp_correction`         | double | 0.5     | Proportional gain for odometric corrections in Phase 0.                        |
| `phase0_max_correction_vel`    | double | 0.05    | Maximum velocity for odometric corrections in Phase 0 (m/s).                   |
| `phase0_pose_correction_tolerance` | double | 0.005 | Tolerance for pose corrections in Phase 0 (m).                                 |
| `target_marker_id`             | int    | 10      | ID of the target ArUco marker on the docking station.                          |
| `marker_size`                  | double | 0.10    | Physical size of the ArUco marker's side (meters).                             |
| `goal_offset_z`                | double | 0.99    | Z offset from the marker for the robot's final pose (meters).                  |
| `num_samples_to_average`       | int    | 50      | Number of ArUco pose samples to average in Phase 1.                            |
| `num_samples_for_verification` | int    | 50      | Number of ArUco samples for the verification phase.                            |
| `kp_linear_correction`         | double | 0.5     | Proportional gain for linear corrections based on ArUco.                       |
| `max_linear_vel`               | double | 0.05    | Maximum linear velocity for ArUco corrections (m/s).                           |
| `pose_correction_tolerance`    | double | 0.005   | Tolerance for ArUco pose corrections (m).                                      |
| `kp_yaw_correction`            | double | 1.0     | Proportional gain for yaw corrections based on ArUco.                          |
| `max_angular_vel`              | double | 0.25    | Maximum angular velocity for ArUco corrections (rad/s).                        |
| `yaw_correction_tolerance`     | double | 0.017   | Tolerance for ArUco yaw corrections (radians, \~1 degree).                      |
| `min_angular_vel`              | double | 0.05    | Minimum angular velocity for yaw corrections (rad/s).                          |
| `strafe_speed`                 | double | -0.05   | Lateral velocity for the final strafe movement (m/s).                          |
| `final_strafe_offset`          | double | 0.05    | Distance offset for the final strafe, relative to the marker's pose (m).       |
| `undocking_distance`           | double | 0.50    | Distance traveled backward during undocking (meters).                          |
| `charge_verification_timeout`  | double | 20.0    | Timeout for charge status verification (seconds).                              |

-----

## 📝 Example Launch File

You can create a `docking.launch.py` file in the `launch/` folder (which you'll need to add to your structure if it doesn't exist) to start the `docking_action_server` node with your custom parameters:

```python
# docking.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('ibt_ros2_autodocking')
    # Make sure to create the 'config' folder in the root of your package
    # and place the docking_params.yaml file there
    params_file = os.path.join(pkg_share_dir, 'config', 'docking_params.yaml')

    return LaunchDescription([
        Node(
            package='ibt_ros2_autodocking',
            executable='docking_server_node',
            name='docking_action_server',
            output='screen',
            parameters=[params_file]
        ),
        # You can add the client here for testing, if you want to launch it automatically
        # Node(
        #     package='ibt_ros2_autodocking',
        #     executable='docking_client_node',
        #     name='docking_client',
        #     output='screen',
        #     arguments=['dock'] # or 'undock' or 'percentage'
        # )
    ])
```

-----

## 🤝 Contribution

Contributions are welcome\! If you find bugs, have suggestions for improvements, or wish to add new features, feel free to open an [issue](https://www.google.com/search?q=https://github.com/YourOrg/ibt_ros2_autodocking/issues) or a [pull request](https://www.google.com/search?q=https://github.com/YourOrg/ibt_ros2_autodocking/pulls).

-----

## 📧 Contacts

  * **Maintainer:** hhakim
  * **Email:** hhakim815@gmail.com

-----

## 📄 License

This project is released under the Apache 2.0 License. You can find the full text of the license in the [LICENSE](https://github.com/Vor7reX/ibt_ros2_autodocking/edit/main/LICENSE) file in the root of this repository.


