# ROS2 launch checker

ROS2 are made of different artifacts, and there are some name matchings
among those files that can fail. This script aims to verify them.

Given a ROS2 package, it checks that all the  

**WARNING: this script executes your setup.py files, if any. Use with
caution and under your risk.**

## Installation

```bash
pip install -r requirements.txt
```

## Execution

Requires a path to the root of a ROS2 package (i.e. where the `package.xml`
file is placed). Accepts an optional argument ``--verbose`` or `-v` to display
more details.

Simple version:

```bash
python3 launch_cmake_checker.py ../ros2_tf_stresser-master 
```

Detailed version:

```bash
python3 launch_cmake_checker.py ../ros2_tf_stresser-master --verbose
```
