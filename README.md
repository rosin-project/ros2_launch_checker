# ROS2 launch checker
![Tests](https://github.com/rosin-project/ros2_launch_checker/workflows/Tests/badge.svg)

ROS2 packages are made of different artifacts, and there are some name matchings
among those files that can fail. This script aims to verify them.

Given a ROS2 package, it checks that all the `node_executable`s on any launch
files are correctly defined either in a `CMakeLists.txt` file or in the 
`setup.py` file. If there is any mismatch, name suggestions are provided. 

**WARNING: this script executes files called `setup.py`. Use under your
own risk.**

## Installation

```bash
pip install .
```

## Execution

Requires a path to the root of a ROS2 package (i.e. where the `package.xml`
file is placed). Accepts an optional argument ``--verbose`` or `-v` to display
more details.

This script runs with python 3.

Simple version:

```bash
python3 -m launch_cmake_checker ../ros2_tf_stresser-master 
```

Verbose version:

```bash
python3 -m launch_cmake_checker ../ros2_tf_stresser-master --verbose
```


## Testing

Some tests are included. In the root folder, run the following:

```bash
python3 -m unittest discover -b
```

The `-b` option supresses the standard input logging information.
