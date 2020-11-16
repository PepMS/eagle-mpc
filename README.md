# Multicopter MPC
This is a C++ libary to solve optimal control problems for multirotor-based robotic systems.

It is based on [Crocoddyl](https://github.com/loco-3d/crocoddyl).

It contains Python bindings, only tested with Python3 version.

## Building
### Install Crocoddyl

Choose any of the options expaplined in the [Crocoddyl repository](https://github.com/loco-3d/crocoddyl).

### Build from source

Clone this repo.

#### YAML-Parser

This will eventually be placed outside this repo. However, for the time being it is placed in the same folder.
```bash
cd path/to/multicopter_mpc
cd yaml_parser
mkdir build && cd build
cmake ..
make -j6
sudo make install
``` 

#### Multicopter MPC

This will eventually be placed outside this repo. However, for the time being it is placed in the same folder.
```bash
cd path/to/multicopter_mpc
mkdir build && cd build
cmake ..
make -j6
sudo make install
``` 

## Running examples
As this library contains Python bindings to its C++ code, we can run a python-based example.
```
python3 path/to/multicopter_mpc/examples/python/trajectory_generation.py
```
