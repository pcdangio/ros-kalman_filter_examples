# kalman_filter_examples

This package includes several examples on how to use the [kalman_filter](https://github.com/pcdangio/ros-kalman_filter) package.

Examples are provided for the following filters:

- **Kalman Filter**
- **Unscented Kalman Filter**
- **Unscented Kalman Filter - Augmented**

Each filter contains the following examples:

- **minimal**: A basic minimum example of how to use the filter's library.
- **simple**: A simplistic demonstration of how to use the filter in a simulated model.
- **profile**: A program for measuring the speed of the filter's calculations.

Examples can be found under the src/ directory.

This package's [CMakeLists.txt](https://github.com/pcdangio/ros-kalman_filter_examples/blob/main/CMakeLists.txt) may also be used as an example for linking your package against the `kalman_filter` package.

## Table of Contents

- [Installation](#1-installation): Instructions for installing the package from source.
- [Usage](#2-usage): Instructions for using the various examples.

## 1: Installation

**Dependencies:**

- ROS Melodic or higher
- Eigen 3 (comes with ROS)

**Download/Build:**

You may use the following commands to clone and build the package:

```bash
# Switch to your catkin workspace source folder.
cd catkin_workspace/src

# Clone package into source folder.
git clone https://github.com/pcdangio/ros-kalman_filter_examples.git kalman_filter_examples

# Switch back to catkin workspace root folder.
cd ..

# Build with catkin_make.
# NOTE: Using "-DCMAKE_BUILD_TYPE=Release" significantly improves performance of the kalman filter libraries.
catkin_make --pkg=kalman_filter -DCMAKE_BUILD_TYPE=Release
```

## 2: Usage

After the examples are built, the following executables will be available for running:

- `kf_simple`
- `kf_profile`
- `ukf_simple`
- `ukf_profile`
- `ukfa_simple`
- `ukfa_profile`

You may run any of these with a standard `rosrun` command. For example, to run the kf_simple example:

```bash
rosrun kalman_filter_examples kf_simple
```