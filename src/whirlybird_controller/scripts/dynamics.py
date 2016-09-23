import numpy as np
import param as P



class Dynamics:

    def __init__(self):

        # Initial state conditions
        self.state = np.matrix([[P.phi0],
                                [P.theta0],
                                [P.psi0],
                                [P.phidot0],
                                [P.thetadot0],
                                [P.psidot0]])

    def propagateDynamics(self,u):
        # P.Ts is the time step between function calls.
        # u contains the force and/or torque input(s).

        # RK4 integration
        k1 = self.Derivatives(self.state, u)
        k2 = self.Derivatives(self.state + P.Ts/2*k1, u)
        k3 = self.Derivatives(self.state + P.Ts/2*k2, u)
        k4 = self.Derivatives(self.state + P.Ts*k3, u)
        self.state += P.Ts/6 * (k1 + 2*k2 + 2*k3 + k4)


    # Return the derivatives of the continuous states
    def Derivatives(self,state,u):

        # States and forces
        phi = state.item(0)
        theta = state.item(1)
        psi = state.item(2)
        phidot = state.item(3)
        thetadot = state.item(4)
        psidot = state.item(5)
        fl = u[0]
        fr = u[1]

        # ctheta and stheta are used multiple times. They are
        # precomputed and stored in another variable to increase
        # efficiency.
        cp = np.cos(phi)
        sp = np.sin(phi)
        ct = np.cos(theta)
        st = np.sin(theta)

        # The equations of motion.
        M = np.matrix([[P.Jx, 0, -P.Jx*st],
                       [0, P.m1*P.l1*P.l1+P.m2*P.l2*P.l2+P.Jy*cp*cp+P.Jz*sp*sp, (P.Jy-P.Jz)*sp*cp*ct],
                       [-P.Jx*st, (P.Jy-P.Jz)*sp*cp*ct, (P.m1*P.l1*P.l1+P.m2*P.l2*P.l2+P.Jy*sp*sp+P.Jz*cp*cp)*ct*ct+P.Jx*st*st]])

        C  = ([[-thetadot*thetadot*(P.Jz - P.Jy)*sp*cp + phidot*phidot*(P.Jz - P.Jy)*sp*cp*ct*ct \
				- thetadot*psidot*ct*(P.Jx - (P.Jz - P.Jy)*(cp*cp - sp*sp))],
		        [phidot*phidot*st*ct*(-P.Jx + P.m1*P.l1*P.l1 + P.m2*P.l2*P.l2 + P.Jy*sp*sp + P.Jz*cp*cp) \
				- 2*phidot*thetadot*(P.Jz - P.Jy)*st*ct - phidot*psidot*ct*(-P.Jx + (P.Jz - P.Jy)*(cp*cp - sp*sp))],
		        [thetadot*thetadot*(P.Jz - P.Jy)*sp*cp*st - phidot*thetadot*ct*(P.Jx + (P.Jz - P.Jy)*(cp*cp - sp*sp)) \
				- 2*phidot*psidot*(P.Jz - P.Jy)*ct*ct*sp*cp + 2*thetadot*psidot*st*ct*(P.Jx - P.m1*P.l1*P.l1 - P.m2*P.l2*P.l2 - P.Jy*sp*sp - P.Jz*cp*cp)]])

        dP = np.matrix([[0],
                        [(P.m1*P.l1-P.m2*P.l2)*P.g*ct],
                        [0]])

        Q = np.matrix([[P.d*(fl-fr)],
                        [P.l1*(fl+fr)*cp],
                        [P.l1*(fl+fr)*ct*sp+P.d*(fr-fl)*st]])

        tmp = np.linalg.inv(M)*(Q - C -dP)


        phiddot = tmp.item(0)
        thetaddot = tmp.item(1)
        psiddot = tmp.item(2)

        xdot = np.matrix([[phidot],[thetadot],[psidot],[phiddot],[thetaddot],[psiddot]])

        return xdot


    # Returns the observable states
    def Outputs(self):
        # Return them in a list and not a matrix
        return self.state[0:3].T.tolist()[0]

    # Returns all current states
    def States(self):
        # Return them in a list and not a matrix
        return self.state.T.tolist()[0]
