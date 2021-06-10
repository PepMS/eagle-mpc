:warning: **Disclaimer** :warning:

This is a work-in-progress library. As such, it only contains basic features. For any doubt, bug, problem or suggestion feel free to open an issue. 

# Eagle MPC
## Introduction
This library contains tools to solve *optimal control problems* (OCPs) that deal with *unmanned aerial manipulators* (UAMs).
It is strongly dependant on [Crocoddyl](https://github.com/loco-3d/crocoddyl), whose API is used to build the OCPs.

It has two principal pieces:
- **Trajectory generator**: It can be used to generate different maneuvers using any type of UAM. The OCP is easily specified by means of a YAML file (see the [example section](#examples))

- **Nonlinear model predictive controllers**: It contains several nMPC controllers. They differ on the way the OCP inside the controller is built and how it is updated at every nMPC step.

It also contains an implementation of the **Squash-box FDDP** solver presented in [this paper](http://www.iri.upc.edu/files/scidoc/2352-Squash-box-feasibility-driven-differential-dynamic-programming.pdf).

This is a C++ library. However, it can also be used within a Python environment since almost all classes have their corresponding **Python bindings**.

## Installation

### Dependencies
#### Crocoddyl

This library depends on the forked version of [Crocoddyl](https://github.com/PepMS/crocoddyl). Its devel branch is up to date. This version allows to choose among different stopping criteria for its different solvers.

Follow its documentation to build from source.

#### example-robot-data
Use the forked version of the [original repository](https://github.com/Gepetto/example-robot-data). 
The [forked version](https://github.com/PepMS/example-robot-data) contains additional UAM models with different platform and robotic arm combination:

**Platforms**:
- Planar small hexacopter (370mm)
- Planar medium-sized hexacopter (680mm)
- Fully actuated hexacopter (Tilthex)

**Robotic Arms**:
- 2 DoFs, 3 DoFs and 5 DoFs (with spherical wrist)

### Eagle MPC

Clone this repo and build:
```bash
git clone https://github.com/PepMS/eagle-mpc.git
cd eagle-mpc
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6
```

Install this library. By default it will be install at `usr/local/`. If you want to install this somewhere else, the `CMAKE_INSTALL_PREFIX` from the `CMakeLists.txt`should be modified accordingly.
```
sudo make install
```

## <a name="examples"></a> Running examples
As this library contains Python bindings to its C++ code, we can run a python-based example.
```
python3 path/to/multicopter_mpc/examples/python/trajectory_generation.py
```
