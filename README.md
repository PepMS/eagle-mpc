:warning: **Disclaimer** :warning:

This is a work-in-progress library. As such, it only contains basic features. For any doubt, bug, problem or suggestion feel free to open an issue.

By now it has only been tested with Ubuntu 20.04 and Python 3.8.

# Eagle MPC
## 1. Introduction
This library contains tools to solve *optimal control problems* (OCPs) that deal with *unmanned aerial manipulators* (UAMs).
It is strongly dependant on [Crocoddyl](https://github.com/loco-3d/crocoddyl), whose API is used to build the OCPs.

It has two principal pieces:
- **Trajectory generator**: It can be used to generate different maneuvers using any type of UAM. The OCP is easily specified by means of a YAML file (see the [example section](#examples))

- **Nonlinear model predictive controllers**: It contains several nMPC controllers. They differ on the way the OCP inside the controller is built and how it is updated at every nMPC step.

It also contains an implementation of the **Squash-box FDDP** solver presented in [this paper](http://www.iri.upc.edu/files/scidoc/2352-Squash-box-feasibility-driven-differential-dynamic-programming.pdf).

This is a C++ library. However, it can also be used within a Python environment since almost all classes have their corresponding **Python bindings**.

## 2. Installation - Dependencies
### 2.1 Crocoddyl dependencies
Crocoddyl has several dependencies. We need to install the following ones:

#### 2.1.1 Pinocchio 
Follow the [installation instructions](https://stack-of-tasks.github.io/pinocchio/download.html). Install from *robotpkg* recommended.

#### 2.1.2 Gepetto viewer: 
Follow the [installation instructions](https://github.com/Gepetto/gepetto-viewer-corba). Install from *robotpkg* recommended.

#### 2.1.3 example-robot-data
Use the [forked version](https://github.com/PepMS/example-robot-data) of the [original repository](https://github.com/Gepetto/example-robot-data). 
The `devel` branch of the forked version contains additional UAM models with different platform and robotic arm combination:

Platforms:
- Planar small hexacopter (370mm)
- Planar medium-sized hexacopter (680mm)
- Fully actuated hexacopter (Tilthex)

Robotic Arms:
- 2 DoFs
- 3 DoFs
- 5 DoFs (with spherical wrist)

Quick installation instructions:
```console
cd <choose-your-path>
git clone https://github.com/PepMS/example-robot-data
cd example-robot-data
git submodule update --init
git checkout devel
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6
sudo make install
```

### 2.2 Crocoddyl
**Crocoddyl** stands for *Contact Robot Optimal Control by Differential Dynamic Library*. It is a library to create and solve optimal control problems for robotics.

:warning: **Crocoddyl needs to be built from source.** :arrow_down::arrow_down:Reason:arrow_down::arrow_down:

*EagleMPC* has *Crocoddyl* as its main dependency (most of the classes contain *Crocoddyl* objects). Unfortunately, we cannot use the release packages given by the *Crocoddyl* team and we need to build it from source.
This is due to the implementation of the *Squash-box FDDP* solver in the *EagleMPC* library. 
It inherits from the base class `crocoddyl::SolverFDDP` that has been slightly modified to consider different stopping criteria. 

The *Crocoddyl* version you need to build is in [this branch](https://github.com/PepMS/crocoddyl/tree/sbfddp-v2).

Overview of the different branches in the [forked Crocoddyl repository](https://github.com/PepMS/crocoddyl):
- **master** : Even with its `master` [parent branch](https://github.com/loco-3d/crocoddyl/tree/master)
- **devel** : Even with its `devel` [parent branch](https://github.com/loco-3d/crocoddyl/tree/devel)
- **sbfddp**: Even with the `devel` branch adding the modifications of the stopping criteria.

Quick installation instructions:
```console
cd <choose-your-path>
git clone https://github.com/PepMS/crocoddyl.git
cd crocoddyl
git submodule update --init
git checkout sbfddp
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6
sudo make install
```

Add the installation folder to your environment variables:
```console
export PATH=/usr/local/bin:$PATH
export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
export PYTHONPATH=/usr/local/lib/python3/dist-packages:$PYTHONPATH
```

 ⚠️ **If you want to reproduce the experiments of the paper** *Full-body torque-level Nonlinear Model Predictive Control*, checkout the *Crocoddyl* repository to [this tag](https://github.com/PepMS/crocoddyl/releases/tag/fbtlnmpc_uam).

### 2.3 YAML-Cpp
YAML files are used to ease the creation of optimal control problems, to help in the definition of new multicopter platforms or to define the tunning parameters for the MPC controllers. 

You can install this dependency by doing:
```console
sudo apt-get install libyaml-cpp-dev
```

## 3. Installation - Eagle MPC

Clone this repo and build:
```console
git clone https://github.com/PepMS/eagle-mpc.git
cd eagle-mpc
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j6
```

Install this library. By default it will be installed at `usr/local/`. If you want to install it somewhere else, the `CMAKE_INSTALL_PREFIX` from the `CMakeLists.txt`should be modified accordingly. Then, to install do
```
sudo make install
```

:warning: **Problem with the system locale** :warning: The Yaml parser uses the `std::stod` function to convert a string to a double. This function is locale dependant. Be sure to have set a locale that uses `.` as a decimal separator. To make sure of it you can run:
```
export LC_NUMERIC="en_US.UTF-8"
```

## <a name="examples"></a> 4. Running examples
As this library contains Python bindings to its C++ code, we can run a python-based example.
```
python3 path/to/multicopter_mpc/examples/python/trajectory_generation.py
```
