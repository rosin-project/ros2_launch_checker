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
pip3 install .
```

## Execution

Requires a path to the root of a ROS2 package (i.e. where the `package.xml`
file is placed). Accepts an optional argument ``--verbose`` or `-v` to display
more details.

This script runs with python 3.

Simple version:

```bash
python3 -m ros2_launch_checker my_great_ros2_package/
```

Verbose version:

```bash
python3 -m ros2_launch_checker my_great_ros2_package/ --verbose
```


## Testing

Some tests are included. In the root folder, run the following:

```bash
python3 -m unittest discover -b
```

The `-b` option supresses the standard input logging information.

## Acknowledgements

<!--
    ROSIN acknowledgement from the ROSIN press kit
    @ https://github.com/rosin-project/press_kit
-->

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" alt="rosin_logo" height="60">
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software Components.
More information: <a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon 2020 research and innovation programme under grant agreement no. 732287.

