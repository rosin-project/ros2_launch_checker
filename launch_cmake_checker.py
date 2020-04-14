import argparse
import os
import re
import difflib
import importlib
import setuptools
from colorama import Fore, Style


class Occurrence:
    """ Wrapper to associate certain string with a file and a line number """
    def __init__(self, file, line_number, content):
        self.file = file
        self.line_number = line_number
        self.content = content


    def totext(self, error=False):
        c = Style.BRIGHT + (Fore.RED if error else Fore.BLUE)
        res = f"{c}{self.content}{Style.RESET_ALL} at {self.file}"
        if self.line_number:
            res += ":" + str(self.line_number)
        return res


    def __str__(self):
        return self.totext()


    def __repr__(self):
        return str(self)


class RosPackage:
    """ Ros package manager through a path """
    def __init__(self, path):
        self.path = path
        self.setup_py_entry_points = []
        self.cmakelists_executables = []
        self.launch_files_executables = []


    def explore_package(self):
        print("Checking " + self.path)
        self.explore_setup_py()
        self.explore_cmakelists()
        self.explore_launch_files()


    def verify_integrity(self):
        logger("Verifying integrity...", forced=True)

        # Executables of launch files should be found either
        # in the cmakelists or in the setup.py 
        finding_containers = self.setup_py_entry_points + self.cmakelists_executables
        error_count = 0
        for launch_occ in self.launch_files_executables:
            exec_node = launch_occ.content
            
            # Find if there is any occurence with the same content as ours.
            # If not, report an error.
            if not [x for x in finding_containers if x.content == exec_node]:
                error_count += 1
                logger("- " + launch_occ.totext(True) + " not found.", indent=1, forced=True)

                # Find something close to relate with
                where_to_find = [x.content for x in finding_containers]
                closest = difflib.get_close_matches(exec_node, where_to_find, n=1)
                if closest:
                    closest = closest[0]
                    closest_occ = [x for x in finding_containers if x.content == closest]
                    if closest_occ:
                        closest_occ = closest_occ[0]
                        logger("  Maybe you meant " + str(closest_occ) + " ?", indent=1, forced=True)

        # Report whether there were errors or not
        if error_count:
            error_message = Fore.RED + str(error_count) + " error(s) found"
        else:
            error_message = Fore.GREEN + "No errors found"

        logger(Style.BRIGHT + error_message + Style.RESET_ALL, forced=True)
            

    def explore_setup_py(self):
        """ Extracts the entry points from a setup.py file """
        """ Warning, this function executes python files! """
        logger("Checking setup.py file(s)...")

        # Finds all setup.py files
        find_expression = f'find {path} -type f -name "setup.py"'
        setup_files = os.popen(find_expression).readlines()

        if not setup_files:
            logger(warning("No setup.py files found"), indent=1, forced=True)

        # Iterate over them
        for setup_file in setup_files:
            setup_file = setup_file.strip()

            logger(setup_file + ":", indent=1)

            def setup(**kwargs):
                global setup_py_tree
                setup_py_tree = kwargs
            
            setuptools.setup = setup
            content = open(setup_file).read()
            exec(content)
            
            # setup_py_tree now contains the tree
            entry_points = setup_py_tree["entry_points"]["console_scripts"]

            for ep in entry_points:
                ep = ep.split("=")[0].strip()
                oc = Occurrence(
                    file = setup_file,
                    line_number = None,
                    content = ep
                )
                self.setup_py_entry_points.append(oc)
                logger(str(oc), indent=2)


    def explore_cmakelists(self):
        """ Finds node executables from launch files """

        logger("Checking CMakeLists.txt...")

        find_expression = f'find {path} -type f -name "CMakeLists.txt"'
        cmakelists = os.popen(find_expression).readlines()

        regex = r'add_executable\(([a-zA-Z_][a-zA-Z0-9_]*)'

        if not cmakelists:
            logger(warning("No CMakeLists.txt files found"), indent=1, forced=True)

        for cmakelist in cmakelists:
            cmakelist = cmakelist.strip()
            logger(cmakelist + ":", indent=1)

            with open(cmakelist) as fp:
                for cnt, line in enumerate(fp):
                    groups = re.findall(regex, line)
                    
                    if groups:
                        oc = Occurrence(
                            file = cmakelist,
                            line_number = cnt,
                            content = groups[0]
                        )
                        self.cmakelists_executables.append(oc)
                        logger(str(oc), indent=2)


    def explore_launch_files(self):
        """ Finds node executables from launch files """

        logger("Checking launch file(s)...")

        find_expression = f'find {path} -type f -name "*.launch.py"'
        launchfiles = os.popen(find_expression).readlines()

        regex1 = r"node_executable='([^']+)'"
        regex2 = r'node_executable="([^"]+)"'

        if not launchfiles:
            logger(warning("No launch files found"), indent=1, forced=True)

        for launchfile in launchfiles:
            launchfile = launchfile.strip()
            logger(launchfile + ":", indent=1)

            with open(launchfile) as fp:
                for cnt, line in enumerate(fp):
                    groups = re.findall(regex1, line)
                    groups += re.findall(regex2, line)
                    
                    if groups:
                        oc = Occurrence(
                            file = launchfile,
                            line_number = cnt,
                            content = groups[0]
                        )
                        self.launch_files_executables.append(oc)
                        logger(str(oc), indent=2)


def logger(s, indent=0, forced=False):
    global verbose
    if verbose or forced:
        if indent == 0:
            print()
        print("\t"*indent + ("- " if indent == 2 else "") + s)


def warning(s):
    return Style.BRIGHT + Fore.YELLOW + s + Style.RESET_ALL


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Checks ROS package inconsistencies'
    )
    parser.add_argument("path", help="the path of the package")
    parser.add_argument("-v", "--verbose", action="store_true")

    args = parser.parse_args()
    path = args.path
    global verbose
    verbose = args.verbose

    rp = RosPackage(path)
    rp.explore_package()
    rp.verify_integrity()