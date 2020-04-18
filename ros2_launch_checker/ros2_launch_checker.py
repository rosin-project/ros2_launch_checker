import argparse

from ros2_launch_checker.RosPackage import RosPackage

if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Checks ROS package inconsistencies'
    )
    parser.add_argument("path", help="the path of the package")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    path = args.path
    verbose = args.verbose

    rp = RosPackage(path, verbose)
    rp.explore_package()
    rp.verify_integrity()