import unittest

class TestStringMethods(unittest.TestCase):

    def test_upper(self):
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
    unittest.main()
# import numpy as np
# import crocoddyl
# # import eigenpy
# import pinocchio

# import eagle_mpc

# knots = 24
# pos = np.array([1, 1, 1])
# quat = pinocchio.Quaternion(1, 0, 0, 0)
# vel = np.array([1, 1, 1])
# rate = np.array([1, 2, 1])

# wp_1 = eagle_mpc.WayPoint(knots, pos, quat)
# wp_2 = eagle_mpc.WayPoint(knots, pos, quat, vel, rate)

# wp_lst = []
# wp_lst.append(wp_1)
# wp_lst.append(wp_2)

# for idx, wp in enumerate(wp_lst):
#     print("Printing Waypoint ", idx)
#     print("Knots", wp.knots)
#     print("Pose \n", wp.pose)
#     if (wp.velocity is not None):
#         print("Velocity \n", wp.velocity)
