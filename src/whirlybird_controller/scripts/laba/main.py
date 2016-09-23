import time
import sys
import numpy as np
from slider_input import Sliders
import matplotlib.pyplot as plt 
import param as P

# The Animation.py file is kept in the parent directory,
# so the parent directory path needs to be added.
sys.path.append('..')
from animation import WhirlybirdAnimation 


t_start = 0.0    # Start time of simulation
t_end = 20.0     # End time of simulation
t_Ts = P.Ts      # Simulation time step
t_elapse = 0.01  # Simulation time elapsed between each iteration
t_pause = 0.01   # Pause between each iteration


# Instantiate classes
user_input = Sliders()                
simAnimation = WhirlybirdAnimation()   

t = t_start     # Declare time variable to keep track of simulation time elapsed

while t < t_end:

	plt.ion()					          # Make plots interactive
	plt.figure(user_input.fig.number)     # Switch current figure to user_input figure
	plt.pause(0.0001)                     # Pause the simulation to detect user input
	plt.figure(simAnimation.fig.number)   # Switch current figure to animation figure
	simAnimation.drawWhirlybird(
		user_input.getInputValues())

	t = t+t_elapse                        # Update animation with current user input
	# time.sleep(t_pause)