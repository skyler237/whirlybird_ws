
import sys
import numpy as np
import param as P

class controllerSS:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the SS_ctrl object
      self.SSCtrl = SS_ctrl(P.K_lon,P.ki_lon,P.K_lat,P.ki_lat,P.phi0,P.theta0,P.psi0)
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


class SS_ctrl:
  def __init__(self,K_lon,ki_lon,K_lat,ki_lat,phi0,theta0,psi0):
      self.phidot = 0.0              # Difference term
      self.thetadot = 0.0          # Difference term
      self.psidot =0.0
      self.integrator_lon = 0.0
      self.integrator_lat = 0.0
      self.phi_d1 = phi0               # Last z term
      self.theta_d1 = theta0       # Last theta term
      self.psi_d1 = psi0
      self.error_theta_d1 = 0.0
      self.error_psi_d1 = 0.0
      self.ki_lon = ki_lon
      self.K_lon = K_lon                   # Closed loop SS gains
      self.ki_lat = ki_lat                 # Input gain
      self.K_lat = K_lat

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

      error_psi = psi_r - psi
      error_theta = theta_r - theta

      # Update Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.phidot = a1*self.phidot + a2*(phi-self.phi_d1)

      self.thetadot = a1*self.thetadot + a2*(theta-self.theta_d1)

      self.psidot = a1*self.psidot + a2*(psi-self.psi_d1)

      self.integrator_lat += (P.Ts/2.0)*(error_psi+self.error_psi_d1)


      self.integrator_lon += (P.Ts/2.0)*(error_theta+self.error_theta_d1)





      self.error_psi_d1 = error_psi
      self.error_theta_d1 = error_theta
      self.phi_d1 = phi
      self.theta_d1 = theta
      self.psi_d1 = psi

      # Construct the state
      x_lat = np.matrix([[phi],
                     [psi],
                     [self.phidot],
                     [self.psidot]])

      x_lon = np.matrix([[theta],
                        [self.thetadot]])

      # Compute the state feedback controller
      F = P.F0 - self.K_lon*(x_lon - P.x0_lon) - self.ki_lon*self.integrator_lon
      T = P.T0 - self.K_lat*(x_lat - P.x0_lat) - self.ki_lat*self.integrator_lat

    #   print "K_lat terms"
    #   print -self.K_lat
    #   print phi
    #   print psi
    #   print self.phidot
    #   print self.psidot
      u = self.convertForces([F,T])
    #   print "U:"
    #   print u[0]
    #   print u[1]

      Fl = u[0]*P.km
      Fr = u[1]*P.km
      F_sat = Fr + Fl
      T_sat = P.d*(Fl-Fr)

    #   print "Force:"
    #   print F
    #   print F_sat

    #   print "Torque:"
    #   print T
    #   print T_sat

    #   if self.ki_lon !=0:
    #     self.integrator_lon += P.Ts/self.ki_lon*(F_sat-F)
    #   if self.ki_lat !=0:
    #     integrator_unwind_lat = P.Ts/self.ki_lat*(T_sat-T)
    #     print "Lat int. unwind:"
    #     print integrator_unwind_lat
    #     self.integrator_lat += integrator_unwind_lat

    #   print "integrators:"
    #   print self.integrator_lon
    #   print self.integrator_lat

      return u
