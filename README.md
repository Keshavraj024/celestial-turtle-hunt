# Celestial Turtle Hunt

![turtle_hunt1](https://github.com/Keshavraj024/celestial-turtle-hunt/assets/45878628/8abcf190-cedf-4a5b-82e4-c3fc6f9256b9)

# Celestial Turtle Hunter

The Celestial Turtle Hunter is a ROS2 (Robot Operating System) project that allows you to hunt and capture celestial turtles that are spawned by the Celestial Turtle Spawner. This README provides an overview of the project's components and how to use them.

## Project Components

### Celestial Turtle Spawner

The `celestial_turtle_spawner` is responsible for spawning celestial turtles into the environment. It is a crucial part of the simulation and provides the turtles that the hunter will attempt to catch.

### Celestial Turtle Interface

The `celestial_turtle_interface` module acts as the interface between different components of the project. It manages the communication between the spawner, controller, and teleoperation modules, ensuring seamless coordination.

### Celestial Turtle Lib

The `celestial_turtle_lib` module contains helper functions and utilities that are used throughout the project.

### Celestial Turtle Controller

The `celestial_turtle_controller` is responsible for controlling the overall project operation. It can shut down all nodes or, when the turtle is caught, terminate the specific turtle's process. It plays a pivotal role in managing the project's lifecycle.

## Celestial Turtle Teleop

The `celestial_turtle_teleop` module allows you to navigate the Celestial Turtle Hunter robot. It provides a user-friendly interface for controlling the hunter's movement, enabling you to pursue and capture celestial turtles effectively.

## Getting Started

To get started with the Celestial Turtle Hunt project, follow these steps:

1. **Clone the Repository:**

   ```bash
   https://github.com/Keshavraj024/celestial-turtle-hunt.git
   cd celestial_turtle_hunt
   ```

2. **Build and Install Dependencies:**

   Ensure you have ROS 2 installed, then build and install the project and its dependencies:

   ```bash
   colcon build
   ```

3. **Launch the Project:**

   Launch the project components using ROS 2 launch files:

   ```bash
   source install/setup.bash
   ros2 launch celestial_turtle_hunter turtle_hunter.launch.xml 
   ros2 launch celestial_turtle_spawner turtle_spawner.launch.xml
   ros2 run celestial_turtle_controller turtle_controller_node
   ```

4. **Control the Hunter:**

   Use the `celestial_turtle_teleop` module to navigate the hunter and pursue celestial turtles.

   ```bash
   ros2 run celestial_turtle_teleop turtle_teleop_node
   ```

5. **Capture Turtles:**

   When you catch a celestial turtle, the `celestial_turtle_controller` can terminate the turtle's process or perform other actions as required.

## Contributing

Contributions to the Celestial Turtle Hunter project are welcome! Feel free to open issues, submit pull requests, or improve the documentation to make this project even better.
