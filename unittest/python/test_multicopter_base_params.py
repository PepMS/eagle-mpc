import numpy as np
import multicopter_mpc

cf = 1e-6
cm = 1e-7
d_cog = 0.1525
tau_f = np.array([[0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0], [1.0, 1.0, 1.0, 1.0], [0.0, d_cog, 0.0, -d_cog],
                  [-d_cog, 0.0, d_cog, 0.0], [-cm / cf, cm / cf, -cm / cf, cm / cf]])
max_th = 6.1
min_th = 0.1
base_link = "base_link"

params = multicopter_mpc.MultiCopterBaseParams(cf, cm, tau_f, max_th, min_th, base_link)

print("cf: \n", params.cf)
print("cm: \n", params.cm)
print("n_rotors: \n", params.n_rotors)
print("tau_f: \n", params.tau_f)
print("max_thrust: \n", params.max_thrust)
print("max_thrust: \n", params.min_thrust)
print("link name: \n", params.base_link_name)