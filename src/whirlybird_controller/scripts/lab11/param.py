# Inverted Pendulum Parameter File
import numpy as np
import control as cnt

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

# Simulation Parameters
Ts = .01
sigma = 0.05

# Initial Conditions
phi0 = 0.0
phidot0 = 0.0
theta0 = 0.0*np.pi/180.0
thetadot0 = 0.0
psi0 = 0.0
psidot0 = 0.0
F0 = (m1*l1-m2*l2)*g/l1*np.cos(theta0)
T0 = 0.0

th_tr = 4.0
th_wn = 2.2/th_tr
th_zeta = .707

s_tr = 4.0
s_wn = 2.2/s_tr
s_zeta = .707

p_tr = 2.0
p_wn = 2.2/p_tr
p_zeta = .5

integrator_pole_lat = -10
integrator_pole_lon = -10

####################################################
#                 State Space
####################################################
error_max = 1                # Max step size,m
theta_max = 70.0 # Max theta

# State Space Equations
# xdot = A*x + B*u
# y = C*x

a = l1/(m1*l1**2+m2*l2**2+Jx)

A_lat = np.matrix([[0.0,0.0,1.0,0.0],
			   [0.0,0.0,0.0,1.0],
			   [0.0,0.0,0.0,0.0],
			   [a*F0,0.0,0.0,0.0]])

B_lat = np.matrix([[0.0],
			   [0.0],
			   [1/Jx],
			   [0.0]])

C_lat = np.matrix([[0.0,1.0,0.0,0.0]])

x0_lat = np.matrix([[phi0],
			   [psi0],
			   [phidot0],
			   [psidot0]])

# Augmented Matrices
A1_lat = np.concatenate((
	np.concatenate((A_lat,np.zeros((4,1))),axis=1),
	np.concatenate((-C_lat,np.matrix([[0.0]])),axis=1)),axis = 0)

B1_lat = np.concatenate((B_lat,np.matrix([[0.0]])),axis = 0)

b = ((m1*l1-m2*l2)*g)/(m1*l1**2+m2*l2**2+Jy)
c = l1/(m1*l1**2+m2*l2**2+Jy)

A_lon = np.matrix([[0.0,1.0],
				   [b*np.sin(theta0),0.0]])

B_lon = np.matrix([[0.0],
				  [c]])

C_lon = np.matrix([[1.0,0.0]])

x0_lon = np.matrix([[theta0],
					[thetadot0]])

# Augmented Matrices
A1_lon = np.concatenate((
	np.concatenate((A_lon,np.zeros((2,1))),axis=1),
	np.concatenate((-C_lon,np.matrix([[0.0]])),axis=1)),axis = 0)

B1_lon = np.concatenate((B_lon,np.matrix([[0.0]])),axis = 0)

# S**2 + alpha1*S + alpha0
p_alpha1 = 2.0*p_zeta*p_wn
p_alpha0 = p_wn**2

# S**2 + alpha1*S + alpha0
s_alpha1 = 2.0*s_zeta*s_wn
s_alpha0 = s_wn**2

# S**2 + alpha1*S + alpha0
th_alpha1 = 2.0*th_zeta*th_wn
th_alpha0 = th_wn**2

# Desired Poles
des_char_poly_lat = np.convolve(np.convolve([1,s_alpha1,s_alpha0],[1,p_alpha1,p_alpha0]),np.poly(integrator_pole_lat))
des_char_poly_lon = np.convolve([1,th_alpha1,th_alpha0],np.poly(integrator_pole_lon))
des_poles_lat = np.roots(des_char_poly_lat)
des_poles_lon = np.roots(des_char_poly_lon)

# Latitudinal Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A1_lat,B1_lat))!=5:
	print("The Latitudinal system is not controllable")
else:
    K1_lat = cnt.acker(A1_lat,B1_lat,des_poles_lat)
    K_lat = K1_lat[0,0:4]
    ki_lat = K1_lat[0,4]

# Longitudinal Controllability Matrix
if np.linalg.matrix_rank(cnt.ctrb(A1_lon,B1_lon))!=3:
	print("The Longitudinal system is not controllable")
else:
    K1_lon = cnt.acker(A1_lon,B1_lon,des_poles_lon)
    K_lon = K1_lon[0,0:2]
    ki_lon = K1_lon[0,2]

print('K_lat: ', K_lat)
print('ki_lat: ', ki_lat)
print('K_lon: ', K_lon)
print('ki_lon: ', ki_lon)
