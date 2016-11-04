import sys
import numpy as np
import param as P

isSaturated = False

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object

      self.thetaCtrl = thetaPD_ctrl(P.th_kp,P.th_kd,P.th_ki,P.theta0)
      self.psiCtrl = psiPD_ctrl(P.s_kp,P.s_kd,P.s_ki,P.psi0)
      self.phiCtrl = phiPD_ctrl(P.p_kp,P.p_kd,P.p_ki,P.phi0)

      # kp is the proportional gain
      # kd is the derivative gain
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
      if ul > 1 or ur > 1 or ul < 0 or ur < 0:
          isSaturated = True
      else:
          isSaturated = False
      ul = 1 if ul > maxPWM else 0 if ul < 0 else ul
      ur = 1 if ur > maxPWM else 0 if ur < 0 else ur
      return [ul,ur]

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      theta_r = y_r[0]
      theta = y[1]
      # thetadot = y[4]

      psi_r = y_r[1]
      psi = y[2]
      # psidot = y[5]

      phi = y[0]
      # phidot = y[3]

      phi_r = self.psiCtrl.psiPD_loop(psi_r,psi)

      Fe = (P.m1*P.l1-P.m2*P.l2)*P.g/P.l1*np.cos(theta)
      F = Fe + self.thetaCtrl.thetaPD_loop(theta_r,theta) # Calculate the force output
      T = self.phiCtrl.phiPD_loop(phi_r,phi)
      u = self.convertForces([F,T])
      return u

class thetaPD_ctrl:
  def __init__(self,kp,kd,ki,theta0):
      self.differentiator = 0.0    # Difference term
      self.integrator = 0.0
      self.theta_d1 = theta0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki


  def thetaPD_loop(self,theta_r,theta):
      # Compute the current error
      error = theta_r - theta

      # UPIDate Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.differentiator = a1*self.differentiator \
                          + a2*(theta -self.theta_d1)

      if abs(self.differentiator) <0.05 and isSaturated == False:
          self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # Update error_d1
      self.error_d1 = error
      self.theta_d1 = theta

      # PD Control to calculate T
      F = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator
      return F

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u

class psiPD_ctrl:
  def __init__(self,kp,kd,ki,psi0):
      self.differentiator = 0.0    # Difference term
      self.integrator = 0.0
      self.psi_d1 = psi0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki

  def psiPD_loop(self,psi_r,psi):
      # Compute the current error
      error = psi_r - psi

      # UPIDate Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.differentiator = a1*self.differentiator \
                          + a2*(psi -self.psi_d1)

      if abs(self.differentiator) <0.05 and isSaturated == False:
          self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # Update error_d1
      self.error_d1 = error
      self.psi_d1 = psi

      # PD Control to calculate T
      phi_r = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator
      return phi_r

class phiPD_ctrl:
  def __init__(self,kp,kd,ki,phi0):
      self.differentiator = 0.0    # Difference term
      self.integrator = 0.0
      self.phi_d1 = phi0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.ki = ki


  def phiPD_loop(self,phi_r,phi):
      # Compute the current error
      error = phi_r - phi

      # UPIDate Differentiator
      a1 = (2*P.sigma - P.Ts)/(2*P.sigma+P.Ts)
      a2 = 2/(2*P.sigma+P.Ts)
      self.differentiator = a1*self.differentiator \
                          + a2*(phi -self.phi_d1)

      if abs(self.differentiator) <0.05 and isSaturated == False:
          self.integrator += (P.Ts/2.0)*(error+self.error_d1)

      # Update error_d1
      self.error_d1 = error
      self.phi_d1 = phi

      # PD Control to calculate T
      T_r = self.kp*error - self.kd*self.differentiator + self.ki*self.integrator
      return T_r
