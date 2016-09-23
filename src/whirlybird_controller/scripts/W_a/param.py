# Inverted Pendulum Parameter File
import numpy as np

# Physical parameters of the inverted pendulum
l1 = 0.85    # Length from pivot to whirlybird, m
l2 = 0.3048  # Length from pivot to end, m
m1 = 0.891   # Mass of the whirlybird, kg
m2 = 1      # Mass of counterbalance, kg
d =  0.178    # Distance to each rotor, m
h = 0.65    # Height of the stand, m
r = 0.12    # Radius of each rotor, m
Jx = 0.0047 # Inertia in x direction, Kg*m^2
Jy = 0.0014 # Inertia in y direction, Kg*m^2
Jz = 0.0041 # Inertia in z direction, Kg*m^2
km = 0  # ???
Sgyro = 8.7266e-5   # , rad
Spixel = 0.05   # , pixel
g = 9.8     # Gravity, m/s**2

# Simulation Parameters
Ts = 0.01

# Initial Conditions
zv0 = 2.0
h0 = 2.0
zt0 = 2.0
theta0 = 0.0*np.pi/180
