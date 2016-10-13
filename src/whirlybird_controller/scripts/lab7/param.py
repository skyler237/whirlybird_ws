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
Sgyro = 8.7266e-5   # , rad
Spixel = 0.05   # , pixel
g = 9.8     # Gravity, m/s**2
PWM = 0.46
km = (m1*l1*g-m2*l2*g)/(l1*2*PWM) #

b0 = l1/(m1*l1**2+m2*l2**2+Jy)

# Simulation Parameters
Ts = 0.01

th_tr = .6
th_wn = 2.2/th_tr
th_zeta = .707

th_kp = th_wn**2/b0
th_kd = 2*th_zeta*th_wn/b0

F_max = 10

# Initial Conditions
phi0 = 0.0
phidot0 = 0.0
theta0 = -15.0*np.pi/180
thetadot0 = 0.0
psi0 = 0.0
psidot0 = 0.0
force0 = (m1*l1-m2*l2)*g/l1*np.cos(theta0)
torque0 = 0.0
