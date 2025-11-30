# Path Smoothing and Trajectory Control in 2D Space

## 1. Code Repository Overview
This ROS2 package implements a complete navigation stack for a differential drive robot (Turtlebot3). It addresses the assignment objective of generating a smooth trajectory from discrete waypoints and ensuring accurate tracking.

### 1.1 Project Structure
The code is self-contained and modular:
*   **`src/planner_node.cpp`**: Handles path generation and smoothing.
*   **`src/controller_node.cpp`**: Handles robot actuation and trajectory tracking.
*   **`CMakeLists.txt`**: Build configuration for ROS2 Humble.

---

## 2. Documentation

### 2.1 Setup and Execution Instructions

**Prerequisites:**
*   OS: Ubuntu 22.04 (Jammy Jellyfish)
*   ROS2 Distribution: Humble Hawksbill
*   Simulator: `ros-humble-turtlebot3-gazebo`

**Build Steps:**
```bash
# 1. Create a workspace (if not exists)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# 2. Clone/Place this package folder 'trajectory_assignment' here
    ```bash
    git clone https://github.com/coder-nayan07/Trajectory_assignment_be10x.git
    ```   
# 3. Build the package
cd ~/ros2_ws
colcon build --packages-select trajectory_assignment

# 4. Source the overlay
source install/setup.bash
```

**Execution Steps:**
Open 3 separate terminals and run the following commands:

*   **Terminal 1 (Simulation):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    export TURTLEBOT3_MODEL=burger
    ros2 launch turtlebot3_gazebo empty_world.launch.py
    ```

*   **Terminal 2 (Path Planner):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run trajectory_assignment planner_node
    ```

*   **Terminal 3 (Controller):**
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 run trajectory_assignment controller_node
    ```

---

### 2.2 Design Choices and Algorithms

**Architecture:**
The system adopts a loosely coupled **Publisher/Subscriber** architecture.
*   The **Planner** acts as a global path generator, publishing standard `nav_msgs/Path` messages.
*   The **Controller** acts as a local tracker, subscribing to the path and the robot's `nav_msgs/Odometry`.
*   This separation allows for easy debugging (visualizing the path in RViz without running the controller) and modular replacement of algorithms.

**Algorithm 1: Path Smoothing (Catmull-Rom Spline)**
*   **Choice:** I chose Catmull-Rom Splines over Bezier curves.
*   **Reasoning:** In robotics, waypoints often represent critical locations (corners, doors). A Catmull-Rom spline guarantees the path passes **through** every control point, whereas Bezier curves only approximate them.
*   **Implementation:** The algorithm takes 4 points ($P_0, P_1, P_2, P_3$) and interpolates between $P_1$ and $P_2$. I upsample the path to 20 points per segment to ensure smooth velocity profiles.

**Algorithm 2: Trajectory Tracking (Pure Pursuit)**
*   **Choice:** I implemented a geometric Pure Pursuit controller.
*   **Reasoning:** PID controllers can be unstable on geometric paths. Pure Pursuit calculates the curvature required to reach a specific "lookahead point," resulting in smoother motion that naturally handles corners.
*   **Implementation:**
    1.  Find the closest path index to the robot.
    2.  Search forward along the path for a point at distance $L$ (Lookahead Distance).
    3.  Compute steering angle $\alpha$ based on the geometric relationship to that point.

---

### 2.3 Extension to a Real Robot
To deploy this system on a physical Turtlebot3:

1.  **Hardware Interface:** Replace the Gazebo launch command with the physical robot driver:
    `ros2 launch turtlebot3_bringup robot.launch.py`.
2.  **State Estimation:** Real robots do not have perfect Odometry. Wheel slip causes drift.
    *   **Solution:** I would launch **AMCL** (Adaptive Monte Carlo Localization) or **SLAM Toolbox** to provide a corrected map-to-odom transform.
    *   **Code Change:** The controller would need to listen to `tf` frames (`map` -> `base_link`) rather than the raw `/odom` topic.
3.  **Safety Layer:** A simulated robot can crash safely; a real one cannot.
    *   **Solution:** Introduce a standard multiplexer (`cmd_vel_mux`). A safety node reading the Lidar (`/scan`) would have higher priority and publish zero-velocity commands if obstacles are within 0.2m.

---

### 2.4 Extra Credit: Obstacle Avoidance Strategy
To extend this solution to handle dynamic obstacles (without using the full Navigation2 stack), I would implement a **Local Trajectory Rollout (DWA-lite)** approach:

1.  **Perception:** Subscribe to the `/scan` topic to build a local Occupancy Grid (Costmap).
2.  **Trajectory Generation:** Instead of calculating one steering command, generate 20 candidate trajectories for the next $t$ seconds with varying angular velocities.
3.  **Scoring:** For each candidate trajectory:
    *   **Cost =** (Distance to Global Path) + (Inverse Distance to closest obstacle).
    *   Any trajectory that intersects an obstacle in the costmap gets infinite cost.
4.  **Selection:** The controller executes the lowest-cost trajectory. This allows the robot to "deviate" from the spline to avoid a box, then return to the path.
```
