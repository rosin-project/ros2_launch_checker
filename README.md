# ROS2 launch checker

ROS2 packages are made of different artifacts, and there are some name matchings
among those files that can fail. This script aims to verify them.

Given a ROS2 package, it checks that all the `node_executable`s on any launch
files are correctly defined either in a `CMakeLists.txt` file or in the 
`setup.py` file. If there is any mismatch, name suggestions are provided. 

**WARNING: this script executes files called `setup.py`, if any. Use under your
own risk.**

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

Verbose version:

```bash
python3 launch_cmake_checker.py ../ros2_tf_stresser-master --verbose
```


## Testing

Some tests are included. In the root folder, run the following:

```bash
python3 -m unittest -b
```

The `-b` option supresses the standard input logging information.