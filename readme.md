
# Assignment1 - ROS Project

This project implements a ROS-based solution involving two turtles in the `turtlesim` simulation. It includes the following functionalities:

- **UI Node**: Enables user interaction to control turtles by setting velocities and publishing the name of the last-moving turtle.
- **Distance Node**: Monitors the distance between the turtles and publishes a warning if they get too close to the boundaries.

---

## Table of Contents

- [Setup and Requirements](#setup-and-requirements)
- [Project Structure](#project-structure)
- [Nodes Description](#nodes-description)
  - [UI Node](#ui-node)
  - [Distance Node](#distance-node)
- [How to Run the Project](#how-to-run-the-project)
- [Example Outputs](#example-outputs)

---

## Setup and Requirements

### Prerequisites
- ROS (tested on `Noetic` or `Melodic`)
- `catkin_make` build system
- `turtlesim` package installed (`sudo apt-get install ros-<distro>-turtlesim`)

### Build Instructions
1. Clone this repository into your catkin workspace's `src` directory:
   ```bash
   cd ~/catkin_ws/src
   git clone <repository-url>
   ```
2. Build the workspace:
   ```bash
   cd ~/catkin_ws
   catkin_make
   ```
3. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

---

## Project Structure

```plaintext
Assignment1/
├── CMakeLists.txt
├── include/
│   ├── Assignment1/
│   │   ├── UI_Include.h
│   │   ├── Distance_Include.h
├── src/
│   ├── UI.cpp
│   ├── Distance.cpp
├── package.xml
└── README.md
```

- **`include/Assignment1/`**: Contains the header files for the UI and Distance functionalities.
- **`src/`**: Contains the implementation of the nodes.

---

## Nodes Description

### UI Node

- **Purpose**: 
  - Allows the user to control two turtles (`turtle1` and `turtle2`).
  - Publishes the name of the last-moving turtle.
- **Topics Used**:
  - Publishes: `/turtle1/cmd_vel`, `/turtle2/cmd_vel`, `/last_moving_turtle`
  - Subscribes: `/turtle1/pose`, `/turtle2/pose`

### Distance Node

- **Purpose**:
  - Computes the distance between `turtle1` and `turtle2`.
  - Publishes a warning when either turtle approaches the boundary.
- **Topics Used**:
  - Subscribes: `/turtle1/pose`, `/turtle2/pose`
  - Publishes: `/boundary_warning`

---

## How to Run the Project

1. Launch the `turtlesim` simulator:
   ```bash
   rosrun turtlesim turtlesim_node
   ```

2. Run the `UI` node:
   ```bash
   rosrun Assignment1 UI
   ```

3. Run the `Distance` node:
   ```bash
   rosrun Assignment1 Distance
   ```

4. Interact with the turtles:
   - The `UI` node will prompt you to control the turtles.
   - Observe the console for boundary warnings from the `Distance` node.

---

## Example Outputs

- **UI Node Interaction**:
  ```plaintext
  Select turtle (turtle1/turtle2): turtle1
  Enter linear velocity: 2.0
  Enter angular velocity: 0.5
  ```

- **Distance Node Warning**:
  ```plaintext
  WARNING: turtle1 is near the boundary!
  ```

---

Feel free to add more details if needed, such as known issues, future improvements, or any additional dependencies.
