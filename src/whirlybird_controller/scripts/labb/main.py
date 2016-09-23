import time
import sys
import numpy as np
import matplotlib.pyplot as plt 
import param as P
from slider_input import Sliders


# The Animation.py file is kept in the parent directory,
# so the parent directory path needs to be added.
sys.path.append('..')
from dynamics import WhirlybirdDynamics 
from animation import WhirlybirdAnimation 

# Converts force and torque into the left and 
# right forces produced by the propellers.
def convertForces(u):
	F = u[0]         # Force, N
	tau = u[1]       # Torque, Nm
	# Convert Force and Torque to fl and fr
	# fl is the force created by the left propeller
	# fr is the force created by the right propeller
	fl = 1.0/2.0*F+1.0/(2*P.d)*tau
	fr = 1.0/2.0*F-1.0/(2*P.d)*tau
	return [fl,fr]


t_start = 0.0    # Start time of simulation
t_end = 20.0     # End time of simulation
t_Ts = P.Ts      # Simulation time step
t_elapse = 0.01  # Simulation time elapsed between each iteration
t_pause = 0.01   # Pause between each iteration


# Instantiate classes
user_input = Sliders()                
simAnimation = WhirlybirdAnimation() 
dynam = WhirlybirdDynamics()  

t = t_start     # Declare time variable to keep track of simulation time elapsed

while t < t_end:

	plt.ion()					          # Make plots interactive
	plt.figure(user_input.fig.number)     # Switch current figure to user_input figure
	plt.pause(0.0001)                     # Pause the simulation to detect user input

	# The dynamics of the model will be propagated in time by t_elapse 
	# at intervals of t_Ts.
	t_temp = t +t_elapse
	while t < t_temp:
		u = convertForces(              # Convert force and torque to fl and fr
			user_input.getInputValues())
		dynam.propagateDynamics(u)      # Propagate the dynamics of the model in time
		t += t_Ts                       # Update time elapsed


	plt.figure(simAnimation.fig.number) # Switch current figure to animation figure
	simAnimation.drawWhirlybird(        # Update animation with current user input
		dynam.Outputs())

	t = t+t_elapse                        # Update animation with current user input
	# time.sleep(t_pause)
