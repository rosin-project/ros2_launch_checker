#!/usr/bin/env python3

from catkin.find_in_workspaces import find_in_workspaces
import os
import sys
from lxml import etree
import re
import rospkg


pip = list(map(lambda package: package.strip(), open(
    'built-in_pip_packages.txt', 'r').readlines()))
rospack = list(map(lambda package: package.strip(), open(
    'built-in_rospack_packages.txt', 'r').readlines()))


def find_launch_dependencies(path):
    error_packages = set()
    packages = os.popen(f'find {path} -type f -name "package.xml"').readlines()
    for package in packages:
        package_path = package.replace('package.xml', '').strip()
        paths = os.popen(
            f"find {package_path} -type f -name '*.launch'").readlines()

        for p in paths:
            try:
                tree = etree.parse(p.strip())
                includes = tree.xpath('//include/@file')
                for include in includes:
                    regex = re.compile(r'(\$\(find )(\w*)\)', re.IGNORECASE)
                    match = regex.match(include)
                    if not match or len(match.regs) is not 3:
                        continue
                    dependency = match[2]
                    package_dependencies = parse_package(package)
                    if (not exists_in_workspace(dependency, packages) and dependency not in package_dependencies and dependency not in pip and dependency not in rospack):
                        error_packages.add(
                            (dependency, p.strip(), package.strip()))
            except:
                continue

    error_packages = list(error_packages)
    return error_packages


def exists_in_workspace(package, packages):
    metapackage_dependencies = parse_metapackage(packages)
    if package in metapackage_dependencies:
        return True
    return False


def parse_metapackage(packages):
    for package in packages:
        tree = etree.parse(package.strip())
        metapackage = tree.xpath('//metapackage')
        if metapackage:
            return parse_depends(tree)


def parse_package(package):
    tree = etree.parse(package.strip())
    metapackage = tree.xpath('//metapackage')
    if metapackage:
        return None
    depends = parse_depends(tree)
    name = tree.xpath('//name')
    depends.append(name[0].text)
    return depends


def parse_depends(tree):
    run_depends = tree.xpath('//run_depend')
    run_depends = list(map(lambda dep: dep.text, run_depends))
    depends = tree.xpath('//depend')
    depends = list(map(lambda dep: dep.text, depends))
    if depends:
        run_depends.extend(depends)
    exec_depends = tree.xpath('//exec_depend')
    exec_depends = list(map(lambda dep: dep.text, exec_depends))
    if exec_depends:
        run_depends.extend(exec_depends)
    return run_depends


def _get_rospack():
    return rospkg.RosPack()


def print_errors(errors):
    if (errors):
        print("Errors found:")
        for (error, launch_file, package) in errors:
            print(
                f"\t'{error}' found in '{launch_file}' \n\tis missing in '{package}'\n")
    else:
        print("No errors found.")


if sys.gettrace() is not None:
    print('Running in DEBUG mode.')
    path = 'Examples/rosdistro/velodyne_simulator/'
    print(f"Checking {path}")
    errors = find_launch_dependencies(path)
    print_errors(errors)

if (len(sys.argv) > 1):
    print(f"Checking {sys.argv[1]}")
    errors = find_launch_dependencies(sys.argv[1])
    print_errors(errors)
