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
# import multicopter_mpc

# cf = 1e-6
# cm = 1e-7
# d_cog = 0.1525
# tau_f = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0], [0.0, d_cog, 0.0, -d_cog],
#                   [-d_cog, 0.0, d_cog, 0.0], [-cm / cf, cm / cf, -cm / cf, cm / cf]])
# max_th = 6.1
# min_th = 0.1
# base_link = "base_link"

# params = multicopter_mpc.MultiCopterBaseParams(cf, cm, tau_f, max_th, min_th, base_link)

# print("cf: \n", params.cf)
# print("cm: \n", params.cm)
# print("n_rotors: \n", params.n_rotors)
# print("tau_f: \n", params.tau_f)
# print("max_thrust: \n", params.max_thrust)
# print("max_thrust: \n", params.min_thrust)
# print("link name: \n", params.base_link_name)