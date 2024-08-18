# Minimum Curvature Trajectory Planning Project

This project focuses on simulating minimum curvature trajectory planning using the Gazebo simulation environment.

## Installation

Follow the installation guides for the necessary components:

### Ubuntu 18.04
You can download and install Ubuntu 18.04 from [here](https://releases.ubuntu.com/18.04/).

### Clone the Repository 
You need to clone the repository to your local machine:

```bash
git clone https://github.com/takahashi-seiryu/Vehicle-Trajectory-Planning.git
```

### ROS1 Melodic and Gazebo
Install ROS1 Melodic by following the steps provided [here](https://wiki.ros.org/melodic/Installation/Ubuntu).

#### Troubleshooting: Gazebo Connection Error with Ignition Fuel
When running Gazebo, you might encounter an SSL certification error with the Ignition Fuel service:

```bash
[Err] [REST.cc:205] Error in REST request
libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
```

**Error Description:** This error indicates that the target host name address is incorrect.

**Solution:**
To resolve this issue, you need to update the Ignition Fuel URL in your configuration file. Open the config file using vim:

```bash
$ vim ~/.ignition/fuel/config.yaml
```

Then, substitute ignitionfuel with ignitionrobotics in the configuration. This adjustment should correct the connection issue, allowing Gazebo to properly communicate with the Ignition services.

#### Troubleshooting: Gazebo Map Spawn Issue
If you are experiencing issues with spawning maps in Gazebo, the model path might not be properly set. To fix this, you need to update your `GAZEBO_MODEL_PATH` environment variable:

```bash
echo 'export GAZEBO_MODEL_PATH=<world directory>:$GAZEBO_MODEL_PATH' >> ~/.bashrc
source ~/.bashrc
```

For example, if your world files are located in `~/catkin_ws/src/f1tenth-sim/world/`, you would use:

```bash
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/f1tenth-sim/world/:$GAZEBO_MODEL_PATH' >> ~/.bashrc
source ~/.bashrc
```

This setup ensures that Gazebo can locate and load the specified maps, resolving issues related to map spawning.

### Cartographer
Installation instructions for Cartographer are available [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html).

#### Troubleshooting: Error while installing dependencies: libabsl-dev not available
When installing Cartographer, you might encounter the following issue:

```bash
user@computer:~/catkin_ws$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
cartographer: [libabsl-dev] defined as "not available" for OS version [bionic]
```

**Solution**: In the Cartographer package (not cartographer_ros!), you need to modify the package.xml file. Remove or comment out line 46 which contains <depend>libabsl-dev</depend>. This step should resolve the dependency issue.

### OSQP C++ Library
#### Prerequisites: Upgrade CMake to Version 3.18.5
Ubuntu 18.04 ships with CMake version 3.10.2 by default, which is not compatible with the OSQP library. Follow these steps to upgrade CMake to version 3.18.5:
```bash
wget https://github.com/Kitware/CMake/releases/download/v3.18.5/cmake-3.18.5.tar.gz
tar -xzvf cmake-3.18.5.tar.gz
cd cmake-3.18.5
```

Then build and install:

```bash
./bootstrap
make
sudo make install
```

Verify the installation by checking the version of CMake:
```bash
cmake --version
```

After upgrading to CMake 3.18.5, there might be issues with the system recognizing the new version due to the presence of the older version (3.10.2). Directly modify system links to point to the new CMake installed in `/usr/local/bin`. First, find and remove existing links.

```bash
which cmake
sudo rm /usr/bin/cmake
```

Then create a new symbolic link

```bash
sudo ln -s /usr/local/bin/cmake /usr/bin/cmake
```

Verify the installation by checking the version of CMake again. You can recognize the new CMake version 3.18.5:
```bash
cmake --version
```

#### Install OSQP Library

OSQP's C++ interface is provided as source code. Clone the OSQP repository using the following command:

```bash
git clone --recursive https://github.com/osqp/osqp
```

Navigate to the OSQP directory and set up the build environment:

```bash
cd osqp
mkdir build
cd build
cmake ..
```

Compile the library and install it using the following commands:

```bash
cmake --build . --config Release
sudo cmake --build . --target install
```

This will compile the OSQP library in the **Release** configuration and install it to the default system location.
