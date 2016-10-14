import sys
import numpy as np
import param as P

class controllerPD:
  ''' This class inherits other controllers in order to organize multiple controllers.'''

  def __init__(self):
      # Instantiates the PD_ctrl object
      self.thetaCtrl = thetaPD_ctrl(P.th_kp,P.th_kd,P.theta0,P.F_max)
      self.psiCtrl = psiPD_ctrl(P.s_kp,P.s_kd,P.psi0)
      self.phiCtrl = phiPD_ctrl(P.p_kp,P.p_kd,P.phi0)
      # kp is the proportional gain
      # kd is the derivative gain
      # y0 is the initial position of the state

  def getForces(self,y_r,y):
      # y_r is the referenced input
      # y is the current state
      theta_r = y_r[0]
      theta = y[1]
      thetadot = y[4]

      psi_r = y_r[1]
      psi = y[2]
      psidot = y[5]

      phi = y[0]
      phidot = y[3]

      phi_r = self.psiCtrl.psiPD_loop(psi_r,psi,psidot)

      Fe = (P.m1*P.l1-P.m2*P.l2)*P.g/P.l1*np.cos(theta)
      F = Fe + self.thetaCtrl.thetaPD_loop(theta_r,theta,thetadot) # Calculate the force output
      T = self.phiCtrl.phiPD_loop(phi_r,phi,phidot)
      return [F,T]

class thetaPD_ctrl:
  def __init__(self,kp,kd,theta0,limit):
      self.differentiator = 0.0    # Difference term
      self.theta_d1 = theta0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain
      self.limit = limit           # Maxiumum theta


  def thetaPD_loop(self,theta_r,theta,thetadot):
      # Compute the current error
      error = theta_r - theta

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      F_r_unsat = self.kp*error - self.kd*thetadot
      return F_r_unsat

  def saturate(self,u):
    if abs(u) > self.limit:
      u = self.limit*np.sign(u)
    return u

class psiPD_ctrl:
  def __init__(self,kp,kd,psi0):
      self.differentiator = 0.0    # Difference term
      self.psi_d1 = psi0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain

  def psiPD_loop(self,psi_r,psi,psidot):
      # Compute the current error
      error = psi_r - psi

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      phi_r = self.kp*error - self.kd*psidot
      return phi_r

class phiPD_ctrl:
  def __init__(self,kp,kd,phi0):
      self.differentiator = 0.0    # Difference term
      self.theta_d1 = phi0               # Delayed y output
      self.error_d1 = 0.0          # Delayed error
      self.kp = kp                 # Proportional control gain
      self.kd = kd                 # Derivative control gain


  def phiPD_loop(self,phi_r,phi,phidot):
      # Compute the current error
      error = phi_r - phi

      # Update error_d1
      self.error_d1 = error

      # PD Control to calculate T
      T_r = self.kp*error - self.kd*phidot
      return T_r
