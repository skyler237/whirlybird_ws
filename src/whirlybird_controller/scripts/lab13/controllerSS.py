
import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K_lon,P.ki_lon,P.L_lon,P.K_lat,P.ki_lat,P.L_lat,P.phi0,P.theta0,P.psi0)
      # K is the closed loop SS gains
      # kr is the input gain
      # y0 is the initial position of the state

  # Converts force and torque into the left and
  # right forces produced by the propellers.
  def convertForces(self,u):
  	F = u[0]         # Force, N
  	tau = u[1]       # Torque, Nm
  	# Convert Force and Torque to fl and fr
  	# fl is the force created by the left propeller
  	# fr is the force created by the right propeller
  	ul = 1.0/(P.km*2.0)*(F+tau/P.d)
  	ur = 1.0/(P.km*2.0)*(F-tau/P.d)
  	u = self.saturatePWM([ul,ur])
  	return u

  # saturate the PWM to ensure that they are within the
  # range 0-1
  def saturatePWM(self,u):
      maxPWM = 0.6
      ul = u[0]
      ur = u[1]
      ul = maxPWM if ul > maxPWM else 0 if ul < 0 else ul
      ur = maxPWM if ur > maxPWM else 0 if ur < 0 else ur
      return [ul,ur]

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      theta_r = y_r[0]
      psi_r = y_r[1]

      phi = y[0]
      theta = y[1]
      psi = y[2]

      u = self.SSCtrl.SS_loop(phi,theta,theta_r,psi,psi_r) # Calculate the force output
      return u

  def getObsStates(self):
    return self.SSCtrl.getObsStates()


class SS_ctrl:
  def __init__(self,K_lon,ki_lon,L_lon,K_lat,ki_lat,L_lat,phi0,theta0,psi0):
      self.xhat_lon = np.matrix([[P.theta0],  # h
                             [0.0]]) # h dot
      self.xhat_lat = np.matrix([[P.phi0],  # z
                             [P.psi0],  # theta
                             [0.0],  # zdot
                             [0.0]]) # theta dot
      self.F_d1 = 0.0
      self.T_d1 = 0.0
      self.integrator_lon = 0.0
      self.integrator_lat = 0.0
      self.error_theta_d1 = 0.0
      self.error_psi_d1 = 0.0
      self.ki_lon = ki_lon
      self.K_lon = K_lon                   # Closed loop SS gains
      self.L_lon = L_lon
      self.ki_lat = ki_lat                 # Input gain
      self.K_lat = K_lat
      self.L_lat = L_lat

  def convertForces(self,u):
  	F = u[0]         # Force, N
  	tau = u[1]       # Torque, Nm
  	# Convert Force and Torque to fl and fr
  	# fl is the force created by the left propeller
  	# fr is the force created by the right propeller
  	ul = 1.0/(P.km*2.0)*(F+tau/P.d)
  	ur = 1.0/(P.km*2.0)*(F-tau/P.d)
  	u = self.saturatePWM([ul,ur])
  	return u

  # saturate the PWM to ensure that they are within the
  # range 0-1
  def saturatePWM(self,u):
      maxPWM = 0.6
      ul = u[0].item(0)
      ur = u[1].item(0)
      ul = maxPWM if ul > maxPWM else 0 if ul < 0 else ul
      ur = maxPWM if ur > maxPWM else 0 if ur < 0 else ur
      return [ul,ur]


  def SS_loop(self,phi,theta,theta_r,psi,psi_r):

      # Lon Observer
      N = 10
      for i in range(N):
        self.xhat_lon += P.Ts/N*(P.A_lon*(self.xhat_lon-P.x0_lon) + \
          P.B_lon*(self.F_d1-P.F0)+\
          self.L_lon*((np.matrix([[theta]]) - P.C_lon*self.xhat_lon)))

      # Lat Observer
      for i in range(N):
        self.xhat_lat += P.Ts/N*(P.A_lat*(self.xhat_lat-P.x0_lat) + \
          P.B_lat*(self.T_d1-P.T0)+\
          self.L_lat*((np.matrix([[psi],[phi]]) - P.C_lat*self.xhat_lat)))

      error_psi = psi_r - self.xhat_lat.item(1)
      error_theta = theta_r - self.xhat_lon.item(0)

      self.integrator_lat += (P.Ts/2.0)*(error_psi+self.error_psi_d1)

      self.integrator_lon += (P.Ts/2.0)*(error_theta+self.error_theta_d1)

      self.error_psi_d1 = error_psi
      self.error_theta_d1 = error_theta

      # Compute the state feedback controller
      F = P.F0 - self.K_lon*(self.xhat_lon - P.x0_lon) - self.ki_lon*self.integrator_lon
      T = P.T0 - self.K_lat*(self.xhat_lat - P.x0_lat) - self.ki_lat*self.integrator_lat

      self.F_d1 = F
      self.T_d1 = T

      u = self.convertForces([F,T])

      return u

  def getObsStates(self):
    return [self.xhat_lat.tolist(),self.xhat_lon.tolist()]
