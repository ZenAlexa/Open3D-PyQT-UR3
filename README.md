# UR3 Robot Kinematics Simulation
![image](https://github.com/user-attachments/assets/4802df81-bd58-4037-93cc-62b33c02ebb1)

## Table of Contents

- [Introduction](#introduction)
- [Features](#features)
- [Installation](#installation)
- [Usage](#usage)
- [Current Limitations](#current-limitations)
- [Known Issues](#known-issues)
- [Future Improvements](#future-improvements)
- [Contributing](#contributing)
- [License](#license)

## Introduction

Welcome to the **UR3 Robot Kinematics Simulation** project! This application aims to provide a visual and interactive simulation of the UR3 robotic arm, enabling users to control individual joints and observe the resulting movements in real-time. Leveraging powerful libraries such as Open3D for 3D visualization and PyQt5 for the graphical user interface (GUI), this project serves as an educational tool for understanding robotic kinematics and transformations.

## Features

- **3D Visualization**: Render the UR3 robot model using Open3D, allowing for real-time visualization of joint movements.
- **Interactive GUI**: Control each of the six joints individually through sliders and spin boxes using PyQt5.
- **Real-Time Updates**: Observe immediate changes in the robot's pose as you manipulate joint angles.
- **Logging System**: Monitor system messages and updates through a dedicated log display within the GUI.
- **Reset Functionality**: Easily reset all joints to their default positions with a single click.

## Installation

### Prerequisites

Ensure you have Python 3.7 or later installed on your system. You can download Python from the [official website](https://www.python.org/downloads/).

### Clone the Repository

```bash
git clone https://github.com/yourusername/UR3-Robot-Kinematics-Simulation.git
cd UR3-Robot-Kinematics-Simulation
```

### Install Dependencies

Install the required Python libraries using `pip`:

```bash
pip install -r requirements.txt
```

**Required Libraries:**

- `open3d`
- `PyQt5`
- `numpy`
- `math`
- `multiprocessing`

### Download 3D Models

Ensure that the UR3 robot STL files are placed in the `Models` directory as specified in the code. Update the file paths in the script if necessary to match your directory structure.

## Usage

Run the main script to launch the simulation:

```bash
python Main.py
```

### GUI Controls

- **Joint Sliders and Spin Boxes**: Adjust the angle of each joint using the sliders or input precise values using the spin boxes. The robot model will update in real-time to reflect these changes.
- **Reset Button**: Click the "重置机器人" (Reset Robot) button to return all joints to their default (0°) positions.
- **Log Area**: Monitor system messages, including successful updates and error notifications, in the log display area.
- **Pose Information**: View the current cumulative transformation matrix of the robot's end-effector in the designated area.

## Current Limitations

While significant progress has been made in developing the UR3 Robot Kinematics Simulation, the project currently has several limitations:

1. **Hierarchical Transformations**:
   - **Implemented**: Each joint's rotation affects its corresponding robot part and all subsequent parts. For example, rotating joint 3 will move joints 3 to 6.
   - **Not Implemented**: Transformations do not account for hierarchical dependencies properly. Rotations may cause discontinuities or unnatural movements in the robot model, leading to a fragmented appearance.

2. **Inverse Kinematics (IK) Integration**:
   - **Implemented**: Initial attempts to integrate the `ikpy` library for inverse kinematics were made.
   - **Not Implemented**: Due to code structure issues and errors (e.g., `NameError: name 'RobotVisualizer' is not defined`), the IK functionality is incomplete and non-operational. As a result, users cannot specify target positions for the end-effector to automatically compute joint angles.

3. **Error Handling**:
   - While basic error logging is in place, more robust error handling mechanisms are needed to gracefully manage unexpected inputs or failures during transformations.

4. **Performance Optimization**:
   - The current implementation may experience lag or reduced performance with complex models or rapid joint manipulations. Optimization strategies have yet to be fully explored.

## Known Issues

1. **NameError in `RobotVisualizer` Class**:
   - **Description**: The application encounters a `NameError` when attempting to assign the `update_robot` method to the `RobotVisualizer` class.
   - **Cause**: This error arises from incorrect ordering or scope of method definitions within the class, leading to references to undefined names.
   - **Solution**: Ensure that all methods are defined within the class scope before attempting to assign or modify class attributes externally. Proper class method definitions and inheritance should be maintained to avoid such issues.

2. **Fragmented Robot Model**:
   - **Description**: Rotating a specific joint causes disjointed movement in the robot model, making the visualization appear fragmented.
   - **Cause**: The current transformation logic resets all meshes to their base states before applying cumulative transformations. This approach may not accurately preserve hierarchical relationships between robot parts.
   - **Solution**: Implement a hierarchical transformation system where each joint's transformation is relative to its parent, ensuring smooth and coherent movements throughout the robot model.

3. **Incomplete IK Functionality**:
   - **Description**: The inverse kinematics feature is non-functional, preventing users from specifying target positions for the robot's end-effector.
   - **Cause**: Integration attempts with the `ikpy` library resulted in code errors and incomplete method implementations.
   - **Solution**: Refactor the code to correctly integrate the IK library, ensuring that the `ikpy` chain is properly defined and that inverse kinematics computations are accurately applied to update joint angles.

## Future Improvements

To enhance the UR3 Robot Kinematics Simulation, the following improvements are planned:

1. **Robust Inverse Kinematics Integration**:
   - Fully integrate the `ikpy` library to enable users to specify target positions for the end-effector, automatically calculating and updating joint angles accordingly.

2. **Hierarchical Transformation System**:
   - Redesign the transformation logic to support hierarchical dependencies, ensuring that joint rotations result in natural and coherent movements of the entire robot model.

3. **Enhanced Error Handling**:
   - Implement more comprehensive error handling to manage unexpected inputs, transformation failures, and IK computation errors gracefully.

4. **Performance Optimization**:
   - Optimize rendering and transformation processes to improve real-time performance, especially when handling complex models or rapid joint manipulations.

5. **User Interface Enhancements**:
   - Expand the GUI to include additional controls, such as saving/loading joint configurations, displaying real-time end-effector positions, and providing visual indicators for joint limits.

6. **Documentation and Tutorials**:
   - Develop detailed documentation and tutorials to assist users in understanding the application's features, limitations, and usage scenarios.

## Contributing

Contributions are welcome! If you encounter issues or have suggestions for improvements, please open an issue or submit a pull request.

1. Fork the repository.
2. Create a new branch: `git checkout -b feature/YourFeature`.
3. Commit your changes: `git commit -m 'Add some feature'`.
4. Push to the branch: `git push origin feature/YourFeature`.
5. Open a pull request.

## License

This project is licensed under the [MIT License](https://opensource.org/licenses/MIT).

---

**Disclaimer**: This project is a work in progress and may contain bugs or incomplete features. Use it at your own risk.
