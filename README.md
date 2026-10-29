# Curvature and Energy based Trajectory Optimization in Unstructured Environments

Trajectory planning that minimizes path curvature and energy (path length) in unstructured environments using the f1tenth Gazebo simulation environment.

**Reference Paper:**  
["Curvature and Energy based Trajectory Optimization in Unstructured Environments."](https://takahashi-seiryu.github.io/assets/pdf/Curvature_and_Energy_based_Trajectory_Optimization_in_Unstructured_Environments.pdf)

## Code Versions

| Branch | Description |
|---|---|
| `exp_small_map` (default) | Code used for the paper experiments (2025-12-20 ~ 2025-12-21): computation-time optimized QP planner, small dataset setup (441 start/goal pairs in the `simulation` map), and wall-clock measurement of the planning time. |
| `main` | Early version, before the computation-time optimization (planning took over 1 s). |

With the computation-time optimization, the average planning time on the small dataset is about **35 ms** on an i7-14700K.


## Installation

Follow the installation guides for the necessary components. If you run into problems, see [Troubleshooting](#troubleshooting).

### Ubuntu 18.04
You can download and install Ubuntu 18.04 from [here](https://releases.ubuntu.com/18.04/).

### Clone the Repository 
You need to clone the repository to your local machine:

```bash
cd ~
git clone https://github.com/JuHojin/Curvature-and-Energy-based-Trajectory-Optimization.git catkin_ws
```

We will refer to the workspace as `catkin_ws` throughout the rest of this README.

### ROS1 Melodic and Gazebo
Install ROS1 Melodic `ros-melodic-desktop` by following the steps provided [here](https://wiki.ros.org/melodic/Installation/Ubuntu).

And, install followings for the additional dependencies:

```bash
sudo apt-get update
sudo apt-get install ros-melodic-tf2-geometry-msgs ros-melodic-ackermann-msgs ros-melodic-joy ros-melodic-map-server ros-melodic-teleop-twist-keyboard ros-melodic-navigation ros-melodic-teb-local-planner* ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-gazebo-ros-control ros-melodic-ackermann-msgs ros-melodic-serial qt4-default ros-melodic-effort-controllers python-scipy
```

### Cartographer
Installation instructions for Cartographer are available [here](https://google-cartographer-ros.readthedocs.io/en/latest/compilation.html).

On Ubuntu 18.04, `rosdep install` may fail for `libabsl-dev` or Qt5 packages. See [Troubleshooting](#troubleshooting).

### OSQP Library
#### Prerequisites: Install CMake Version 3.18.5
Ubuntu 18.04 ships with CMake version 3.10.2 by default, which is not compatible with the OSQP library. We'll add CMake version 3.18.5 to use specifically for building and installing OSQP, while continuing to use CMake 3.10.2 for everything else. The two versions can coexist without replacing the system installation.

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

When building and installing OSQP, temporarily activate CMake 3.18.5 with the following command. This change is temporary and will revert to CMake 3.10.2 when you open a new terminal:

```bash
export PATH=/usr/local/cmake-3.18.5/bin:$PATH
```

We will only use this version (3.18.5) when building and installing OSQP.

#### Prerequisites: Update Eigen3 From Source Code

Eigen3 version 3.3.7 or higher is required.

```bash
cd ~/Downloads/
wget https://gitlab.com/libeigen/eigen/-/archive/3.3.7/eigen-3.3.7.tar.gz
tar -xzvf eigen-3.3.7.tar.gz
```

Install Eigen3

```bash
cd eigen-3.3.7
mkdir build
cd build
cmake ..
sudo make install
```

#### Install OSQP Library

The planner uses the OSQP v0.6 C API (`osqp.h`). Clone the OSQP repository and check out `v0.6.0`:

```bash
cd ~
git clone --recursive https://github.com/osqp/osqp
cd osqp
git checkout v0.6.0
git submodule update --init --recursive
```

Build and install it:

```bash
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
cmake --build . -j$(nproc)
sudo cmake --build . --target install
sudo ldconfig
```

This installs the OSQP library under `/usr/local`.

#### (Optional) osqp-cpp: A C++ Wrapper for OSQP

`my_global_planner` uses the OSQP C API directly, so osqp-cpp is not required. If you want to install it, use commit `4343373`, which is compatible with OSQP v0.6.0 and GCC 7.5.0 on Ubuntu 18.04:

```bash
cd ~
git clone https://github.com/google/osqp-cpp.git
cd osqp-cpp
git checkout 4343373
```

In `CMakeLists.txt`, change the Abseil dependency's `GIT_TAG` from `origin/master` to `20230125.3`, then build and test:

```bash
mkdir build; cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
make test
```

## Usage

**CAUTION:** Please ensure to restart your terminal and use CMake version 3.10.2 before proceeding. Directories generated in previous builds (`devel_isolated/`, `build_isolated/`, `install_isolated/`, `devel/`, `build/`, `install/`) may affect the build process. To clean up these directories, run the following commands in your terminal (to keep a backup instead, see [Troubleshooting](#troubleshooting)):
```bash
cd ~/catkin_ws
rm -r devel_isolated/ build_isolated/ install_isolated/ devel/ build/ install/
```

Finally, Build the project using ninja and verify the vehicle model and map spawn correctly in Gazebo simulation. Build in Release mode with the flags below; without them the planner is built without optimization and the planning time is much longer.

```bash
cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
catkin_make_isolated --use-ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE="-O3 -march=native -DNDEBUG" \
  -DCMAKE_C_FLAGS_RELEASE="-O3 -march=native -DNDEBUG"
source ~/catkin_ws/devel_isolated/setup.bash
roslaunch f1tenth-sim simulator.launch
```

**Note:** `-march=native` enables CPU-specific optimization. Binaries compiled with this option may not be portable to other CPU architectures or generations.

**Optional:** You can add the above sourcing command to your ~/.bashrc file to automatically set up the environment every time a new shell is opened:

```bash
echo "source ~/catkin_ws/devel_isolated/setup.bash" >> ~/.bashrc
```

### Control
TBD

### Monte Carlo Experiment (Small Map Dataset)
The paper experiments run the planner on 441 start/goal pairs (`src/f1tenth-sim/data/start_goal_pair_small.csv`) in the `simulation` map. For each pair, the car is reset to the start point, the planner computes a path to the goal, and the car drives along it. Run the following commands in order, each in a separate terminal:

```bash
roslaunch f1tenth-sim simulator.launch
roslaunch f1tenth-sim localization.launch
roslaunch f1tenth-sim navigation.launch
rosrun f1tenth-sim auto_control_monte.py car_1
roslaunch f1tenth-sim monte_carlo.launch
```

**Time-only mode:** To measure only the planning calculation time without driving, skip `auto_control_monte.py` and run the last command as:

```bash
roslaunch f1tenth-sim monte_carlo.launch time_only:=true
```

- **Planner:** `GAMMA` in `src/my_global_planner/include/constant.h` selects the method. `0` is saved as `QP` and a nonzero value (`1e-8` in the paper) as `QP_E_<GAMMA>`. Rebuild after changing it.
- **Inflation layer:** thick (`inflation_radius: 0.5`) or thin (`inflation_radius: 0.35`) in `src/f1tenth-sim/config/global_costmap_params.yaml`.
- **Results:** saved to `src/f1tenth-sim/data/` with the `_small` suffix.

  | Mode | Files |
  |---|---|
  | Full run (default) | `travel_time_small.csv`, `success_small.csv`, `curvature_peak_small.csv`, `calculation_time_small.csv`, `opt_success_small.csv` |
  | Time-only | `calculation_time_small.csv`, `opt_success_small.csv` |

  The runner resumes from existing results, so move these files away to start a new run.

### Mapping (Optional)
If you want to add a new map, you need to generate `.yaml` and `.pgm` files. First, copy the configuration, launch, and URDF files to the designated directories.

``` bash
cd ~/catkin_ws
cp mapping/my_robot.launch src/cartographer_ros/cartographer_ros/launch
cp mapping/my_robot.lua src/cartographer_ros/cartographer_ros/configuration_files
cp mapping/my_robot.urdf src/cartographer_ros/cartographer_ros/urdf
```

Before launching the simulator, modify the world_name argument in the simulator.launch file to match your new map name. You can find this file at `src/f1tenth-sim/launch/simulator.launch`. Once you have updated the `world_name` argument, you can launch the F1Tenth simulator.

```bash
roslaunch f1tenth-sim simulator.launch 
```

Now, launch the Cartographer node to start mapping.

```bash
roslaunch cartographer_ros my_robot.launch
```
You can control the vehicle using the keyboard with the following command. And Monitor the progress of the mapping process in RViz by checking the `map` topic.

```bash
rosrun f1tenth-sim keyboard_teleop.py car_1
```

Once the mapping is complete, save the new map by running the following command. This will generate a `custom_map.pgm` image file and a corresponding `custom_map.yaml` file in the specified directory.

```bash
rosrun  map_server map_saver -f ~/catkin_ws/src/f1tenth-sim/maps/custom_map
```

Finally, update the `map_filename` argument to `custom_map.yaml` at `src/f1tenth-sim/launch/localization.launch` to use your new map in localization.

## Troubleshooting

The following issues were encountered and resolved while setting up the environment on Ubuntu 18.04 / ROS Melodic. Click each item to expand it.

### Installation

<details>
<summary><b>Cartographer: <code>libabsl-dev</code> not available for bionic</b></summary>

When installing Cartographer dependencies, you might encounter the following issue:

```bash
user@computer:~/catkin_ws$ rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y

ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
cartographer: [libabsl-dev] defined as "not available" for OS version [bionic]
```

**Solution:** In the Cartographer package (not cartographer_ros!), modify the `package.xml` file. Remove or comment out line 46 which contains `<depend>libabsl-dev</depend>`.

</details>

<details>
<summary><b>rosdep: <code>libqt5widgets5t64</code>, <code>libqt5gui5t64</code>, <code>libqt5core5t64</code> not available</b></summary>

During dependency installation, `rosdep` attempted to install packages unavailable on Ubuntu 18.04:

```text
libqt5widgets5t64
libqt5gui5t64
libqt5core5t64
```

Ubuntu 18.04 uses the older package names `libqt5widgets5`, `libqt5gui5`, and `libqt5core5a`.

**Solution:** Create a local rosdep override file:

```bash
sudo mkdir -p /etc/ros/rosdep

sudo tee /etc/ros/rosdep/bionic-qt5.yaml > /dev/null <<'EOF'
libqt5-widgets:
  ubuntu:
    bionic: [libqt5widgets5]
libqt5-gui:
  ubuntu:
    bionic: [libqt5gui5]
libqt5-core:
  ubuntu:
    bionic: [libqt5core5a]
EOF
```

Register the override:

```bash
echo "yaml file:///etc/ros/rosdep/bionic-qt5.yaml ubuntu bionic" \
  | sudo tee /etc/ros/rosdep/sources.list.d/00-bionic-qt5.list
```

Update rosdep and reinstall dependencies:

```bash
rosdep update

cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro=melodic -y
```

</details>

<details>
<summary><b>OSQP: <code>fatal: invalid upstream 'origin/v0.1.8'</code></b></summary>

Building the latest OSQP source fails with the following error while fetching the QDLDL dependency:

```text
fatal: invalid upstream 'origin/v0.1.8'
```

**Solution:** The project uses the OSQP v0.6 API, so check out `v0.6.0` and rebuild as described in [Install OSQP Library](#install-osqp-library):

```bash
cd ~/osqp
git checkout v0.6.0
git submodule update --init --recursive
rm -rf build
```

</details>

<details>
<summary><b>osqp-cpp: <code>This package requires GCC 10 or higher</code></b></summary>

Ubuntu 18.04 uses GCC 7.5.0 by default, but the latest Abseil dependency fetched by osqp-cpp requires GCC 10 or newer.

**Solution:** Use the following combination, which was tested successfully (all 41 tests passed):

- osqp-cpp commit: `4343373`
- Abseil version: `20230125.3` (`GIT_TAG` in osqp-cpp's `CMakeLists.txt`)
- OSQP version: `v0.6.0`

See [(Optional) osqp-cpp](#optional-osqp-cpp-a-c-wrapper-for-osqp). The tested revision does not provide an install rule for its own library, so `sudo make install` is not required. osqp-cpp is optional for this project.

</details>

<details>
<summary><b>Rebuilding a restored or moved workspace</b></summary>

When restoring a ROS workspace from another computer or moving it to a different directory, existing build files may contain outdated absolute paths and cached configuration. Keep the source code and Git history, but regenerate the build directories.

To keep a backup of the previous build artifacts instead of deleting them:

```bash
cd ~/catkin_ws
mv build_isolated ~/build_isolated_old
mv devel_isolated ~/devel_isolated_old
```

Then build again with the Release flags described in [Usage](#usage).

</details>

### Python (Python 2.7 / ROS Melodic)

<details>
<summary><b>Python: <code>ImportError: No module named scipy.linalg</code></b></summary>

Running the automatic controller produces:

```bash
rosrun f1tenth-sim auto_control.py car_1
```

```text
ImportError: No module named scipy.linalg
```

**Solution:** The controller requires `scipy.linalg.solve_continuous_are`. Install SciPy for the Python 2 environment used by ROS Melodic:

```bash
sudo apt update
sudo apt install python-scipy
python -c "from scipy.linalg import solve_continuous_are; print('SciPy OK')"
```

</details>

<details>
<summary><b>Python: <code>ImportError: No module named pandas</code></b></summary>

Running the Monte Carlo experiment produces:

```bash
roslaunch f1tenth-sim monte_carlo.launch
```

```text
ImportError: No module named pandas
```

**Solution:** Install Pandas for Python 2:

```bash
sudo apt install python-pandas
python -c "import pandas as pd; print(pd.__version__)"
```

Ubuntu 18.04 provides Pandas 0.22.0, which is too old for the Monte Carlo scripts. Upgrade it as described in the next item.

</details>

<details>
<summary><b>Python: <code>TypeError: concat() got an unexpected keyword argument 'sort'</code></b></summary>

During the Monte Carlo experiment, path optimization completes successfully, but saving the CSV data fails with:

```text
TypeError: concat() got an unexpected keyword argument 'sort'
```

**Cause:** The scripts use `pd.concat(..., sort=False)`. The `sort` argument was introduced in Pandas 0.23.0, but Ubuntu 18.04 provides Pandas 0.22.0.

**Solution:** Install Pandas 0.24.2 (the last version supporting Python 2.7) in the user environment:

```bash
sudo apt install python-pip
python -m pip --version

python -m pip install --user --ignore-installed --no-deps --only-binary=:all: "pandas==0.24.2"
```

This installs Pandas under `~/.local/lib/python2.7/site-packages/` from a precompiled wheel, without changing the system Pandas, NumPy, or SciPy packages. Verify that the new version is loaded:

```bash
python -c "import pandas as pd; print(pd.__version__); print(pd.__file__)"
```

Verified environment: Python 2.7.17, NumPy 1.13.3, Pandas 0.24.2, pip 9.0.1.

</details>

### Gazebo

<details>
<summary><b>Connection error with Ignition Fuel (SSL certificate)</b></summary>

When running Gazebo, you might encounter an SSL certification error with the Ignition Fuel service:

```bash
[Err] [REST.cc:205] Error in REST request
libcurl: (51) SSL: no alternative certificate subject name matches target host name 'api.ignitionfuel.org'
```

This error indicates that the target host name address is incorrect.

**Solution:** Update the Ignition Fuel URL in your configuration file:

```bash
vim ~/.ignition/fuel/config.yaml
```

Update the URL to the following:

```bash
url: https://api.ignitionrobotics.org
```

</details>

<details>
<summary><b>Map does not spawn</b></summary>

If maps do not spawn in Gazebo, the model path might not be properly set. Update your `GAZEBO_MODEL_PATH` environment variable:

```bash
echo 'export GAZEBO_MODEL_PATH=<world directory>:$GAZEBO_MODEL_PATH' >> ~/.bashrc
source ~/.bashrc
```

For example, if your world files are located in `~/catkin_ws/src/f1tenth-sim/world/`, you would use:

```bash
echo 'export GAZEBO_MODEL_PATH=~/catkin_ws/src/f1tenth-sim/world/:$GAZEBO_MODEL_PATH' >> ~/.bashrc
source ~/.bashrc
```

</details>
