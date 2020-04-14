import unittest
import os

from ros2_launch_checker.RosPackage import RosPackage


class PositiveTest(unittest.TestCase):
    def setUp(self):
        pass

    @staticmethod
    def get_test_folder(test_folder):
        this_dir = os.path.dirname(os.path.abspath(__file__))
        return os.path.join(this_dir, os.pardir, test_folder)

    def test_positive(self):
        """ Checks test_positive_source with no problems"""
        rp = RosPackage(self.get_test_folder("test/test_positive_source"), verbose=True)
        rp.explore_package()
        errors = rp.verify_integrity()
        self.assertEqual(errors, [])

    def test_negative(self):
        """ Checks test_negative_source which has
            an incorrect node_executable on five_tree.launch
            it is `stressing` instead of `stressed`
        """
        rp = RosPackage(self.get_test_folder("test/test_negative_source"), verbose=True)
        rp.explore_package()
        errors = rp.verify_integrity()
        self.assertEqual(len(errors), 1)

        error = errors[0]
        self.assertEqual(error.content, "stressing")


if __name__ == "__main__":
    unittest.main()
