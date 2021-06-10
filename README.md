:warning: **Disclaimer** :warning:
This is an work-in-progress library. As such, it contains basic features. For any doubt, problem or suggestion feel free to open an issue. 

# Eagle MPC
This library contains tools to solve optimal control problems that deal with *unmanned aerial manipulators* (UAMs).
It is strongly dependant on [Crocoddyl](https://github.com/loco-3d/crocoddyl), whose API is used to build optimal control problems.

It has two principal pieces:
- **Trajectory generator**: It can be used to generate different maneuvers using any type of UAM. The optimal control problem can be specified by means of a YAML file (see the [example section](#examples))


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

## <a name="examples"></a> Running examples
As this library contains Python bindings to its C++ code, we can run a python-based example.
```
python3 path/to/multicopter_mpc/examples/python/trajectory_generation.py
```
