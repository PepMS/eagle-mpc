import sys
import unittest

from eagle_mpc.utils import (TrajectoryGeneratorDerived)
from eagle_mpc.utils.path import MULTICOPTER_MPC_MISSION_DIR, MULTICOPTER_MPC_MULTIROTOR_DIR

class TrajectoryGeneratorTestCase(unittest.TestCase):
    TRAJECTORY = None
    TRAJECTORY = None

    def test_upper(self):
        eagle_mpc.
        self.assertEqual('foo'.upper(), 'FOO')

    def test_isupper(self):
        self.assertTrue('FOO'.isupper())
        self.assertFalse('Foo'.isupper())

    def test_split(self):
        s = 'hello world'
        self.assertEqual(s.split(), ['hello', 'world'])
        # check that s.split fails when the separator is not a string
        with self.assertRaises(TypeError):
            s.split(2)


if __name__ == '__main__':
    test_classes_to_run = [TrajectoryGeneratorTestCase]
    loader = unittest.TestLoader()
    suites_lst = []
    for test_class in test_classes_to_run:
        suite = loader.loadTestsFromTestCase(test_class)
        suites_lst.append(suite)
    main_suite = unittest.TestSuite(suites_lst)
    runner = unittest.TextTestRunner()
    results = runner.run(main_suite)
    sys.exit(not results.wasSuccessful())
