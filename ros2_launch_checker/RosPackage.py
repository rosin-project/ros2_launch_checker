import os
import re
import difflib
import setuptools
from colorama import Fore, Style

from ros2_launch_checker.Occurrence import Occurrence


class RosPackage:
    """ Ros package manager through a path """

    def __init__(self, path, verbose=False):
        self.path = path
        self.setup_py_entry_points = []
        self.cmakelists_executables = []
        self.launch_files_executables = []
        self.verbose = verbose

    def explore_package(self):
        print("Checking " + self.path)
        self.explore_setup_py()
        self.explore_cmakelists()
        self.explore_launch_files()

    def verify_integrity(self):
        self.logger("Verifying integrity...", forced=True)

        # Executables of launch files should be found either
        # in the cmakelists or in the setup.py
        finding_containers = self.setup_py_entry_points + self.cmakelists_executables
        errors = []
        for launch_occ in self.launch_files_executables:
            exec_node = launch_occ.content

            # Find if there is any occurrence with the same content as ours.
            # If not, report an error.
            if not [x for x in finding_containers if x.content == exec_node]:
                self.logger(
                    "- " + launch_occ.totext(True) + " not found.",
                    indent=1,
                    forced=True,
                )
                errors.append(launch_occ)

                # Find something close to relate with
                where_to_find = [x.content for x in finding_containers]
                closest = difflib.get_close_matches(exec_node, where_to_find, n=1)
                if closest:
                    closest = closest[0]
                    closest_occ = [
                        x for x in finding_containers if x.content == closest
                    ]
                    if closest_occ:
                        closest_occ = closest_occ[0]
                        self.logger(
                            "  Maybe you meant " + str(closest_occ) + " ?",
                            indent=1,
                            forced=True,
                        )

        # Report whether there were errors or not
        if errors:
            error_message = Fore.RED + str(len(errors)) + " error(s) found"
        else:
            error_message = Fore.GREEN + "No errors found"

        self.logger(Style.BRIGHT + error_message + Style.RESET_ALL, forced=True)
        return errors

    def explore_setup_py(self):
        """ Extracts the entry points from a setup.py file """
        """ Warning, this function executes python files! """
        self.logger("Checking setup.py file(s)...")

        # Finds all setup.py files
        find_expression = f'find {self.path} -type f -name "setup.py"'
        setup_files = os.popen(find_expression).readlines()

        if not setup_files:
            self.logger(self.warning("No setup.py files found"), indent=1, forced=True)

        # Iterate over them
        for setup_file in setup_files:
            setup_file = setup_file.strip()

            self.logger(setup_file + ":", indent=1)

            def setup(**kwargs):
                global setup_py_tree
                setup_py_tree = kwargs

            setuptools.setup = setup
            content = open(setup_file).read()
            exec(content)

            # setup_py_tree now contains the tree
            if (
                "entry_points" in setup_py_tree
                and "console_scripts" in setup_py_tree["entry_points"]
            ):
                entry_points = setup_py_tree["entry_points"]["console_scripts"]
            else:
                entry_points = []

            for ep in entry_points:
                ep = ep.split("=")[0].strip()
                oc = Occurrence(file=setup_file, line_number=None, content=ep)
                self.setup_py_entry_points.append(oc)
                self.logger(str(oc), indent=2)

    def explore_cmakelists(self):
        """ Finds node executables from launch files """

        self.logger("Checking CMakeLists.txt...")

        find_expression = f'find {self.path} -type f -name "CMakeLists.txt"'
        cmakelists = os.popen(find_expression).readlines()

        regex = r"add_executable\(([a-zA-Z_][a-zA-Z0-9_]*)"

        if not cmakelists:
            self.logger(
                self.warning("No CMakeLists.txt files found"), indent=1, forced=True
            )

        for cmakelist in cmakelists:
            cmakelist = cmakelist.strip()
            self.logger(cmakelist + ":", indent=1)

            with open(cmakelist) as fp:
                for cnt, line in enumerate(fp):
                    groups = re.findall(regex, line)

                    if groups:
                        oc = Occurrence(
                            file=cmakelist, line_number=cnt, content=groups[0]
                        )
                        self.cmakelists_executables.append(oc)
                        self.logger(str(oc), indent=2)

    def explore_launch_files(self):
        """ Finds node executables from launch files """

        self.logger("Checking launch file(s)...")

        # find_expression = f'find {self.path} -type f -name "*.launch.py"'
        find_expression = (
            f'find {self.path} -type f \( -name "*.launch.py" -o -name "*launch.py" \)'
        )
        launchfiles = os.popen(find_expression).readlines()

        regex1 = r"(?:node_)?executable='([^']+)'"
        regex2 = r'(?:node_)?executable="([^"]+)"'

        if not launchfiles:
            self.logger(self.warning("No launch files found"), indent=1, forced=True)

        for launchfile in launchfiles:
            launchfile = launchfile.strip()
            self.logger(launchfile + ":", indent=1)

            with open(launchfile) as fp:
                for cnt, line in enumerate(fp):
                    groups = re.findall(regex1, line)
                    groups += re.findall(regex2, line)

                    if groups:
                        oc = Occurrence(
                            file=launchfile, line_number=cnt, content=groups[0]
                        )
                        self.launch_files_executables.append(oc)
                        self.logger(str(oc), indent=2)

    @staticmethod
    def warning(s):
        return Style.BRIGHT + Fore.YELLOW + s + Style.RESET_ALL

    def logger(self, s, indent=0, forced=False):
        if self.verbose or forced:
            if indent == 0:
                print()
            print("\t" * indent + ("- " if indent == 2 else "") + s)
