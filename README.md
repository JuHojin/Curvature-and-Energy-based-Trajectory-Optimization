# Minimum Curvature Trajectory Planning Project

This project focuses on simulating minimum curvature trajectory planning using the Gazebo simulation environment.

## Installation

Follow the installation guides for the necessary components:

### Ubuntu 18.04
You can download and install Ubuntu 18.04 from [here](https://releases.ubuntu.com/18.04/).

### Clone the Repository 
You need to clone the repository to your local machine:

```bash
cd ~
git clone https://github.com/takahashi-seiryu/Vehicle-Trajectory-Planning-Using-Gazebo.git
```

**Optional:** Then rename the workspace to `catkin_ws`. You can choose another name, but we will refer to the workspace as `catkin_ws` throughout the rest of this README.

```bash
mv Vehicle-Trajectory-Planning-Using-Gazebo catkin_ws
```

### ROS1 Melodic and Gazebo
Install ROS1 Melodic `ros-melodic-desktop` by following the steps provided [here](https://wiki.ros.org/melodic/Installation/Ubuntu).

And, install followings for the additional dependencies:

```bash
sudo apt-get update
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server ros-melodic-teleop-twist-keyboard ros-melodic-navigation ros-melodic-teb-local-planner* ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-ackermann-msgs ros-melodic-serial qt4-default ros-melodic-effort-controllers
```

Also, we will use ninja for faster builds.
```bash
sudo apt-get install ninja-build
```

Verify the vehicle model and map spawn correctly in Gazebo simulation.

```bash
cd ~/catkin_ws
catkin_make_isolated --use-ninja
source ~catkin_ws/devel_isolated/setup.bash
roslaunch f1tenth-sim simulator.launch
```

#### Troubleshooting: Gazebo Connection Error with Ignition Fuel
When running Gazebo, you might encounter an SSL certification error with the Ignition Fuel service:

```bash
[Err] [REST.cc:205] Error in REST request
libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
```

This error indicates that the target host name address is incorrect.

**Solution:**
You need to update the Ignition Fuel URL in your configuration file. Open the config file using vim:

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
Ubuntu 18.04 ships with CMake version 3.10.2 by default, which is not compatible with the OSQP library. we'll add CMake version 3.18.5 to use specifically for building and installing OSQP, while continuing to use CMake 3.10.2 for everything else. This will allow you to switch between the two versions as needed.

First, download and install CMake 3.18.5:
```bash
cd ~/Downloads
wget https://github.com/Kitware/CMake/releases/download/v3.18.5/cmake-3.18.5.tar.gz
tar -xzvf cmake-3.18.5.tar.gz
cd cmake-3.18.5
```

Then build and install:

```bash
./bootstrap --prefix=/usr/local/cmake-3.18.5
make
sudo make install
```

After installation, if you check the CMake version, it will still show 3.10.2:

```bash
cmake --version
```

If you need to use CMake 3.18.5, for example, when building and installing OSQP, you can temporarily activate it with the following command. Note that this change is temporary and will revert to CMake 3.10.2 when you close the terminal:

```bash
export PATH=/usr/local/cmake-3.18.5/bin:$PATH
```

We will only use this version (3.18.5) when building and installing OSQP.


#### Install OSQP Library

OSQP's C++ interface is provided as source code. Clone the OSQP repository using the following command:

```bash
cd ~
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

This will compile the OSQP library in the Release configuration and install it to the default system location.
