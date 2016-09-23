import matplotlib.pyplot as plt
import sys
import matplotlib.patches as mpatches
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from mpl_toolkits.mplot3d.art3d import Line3DCollection
import numpy as np
import param as P

class Animation:

	def __init__(self):
		self.flagInit=True # Used to indicate initialization
		self.fig = plt.figure()
		self.ax = Axes3D(self.fig) # Create a 3D axes in the figure
		# A list object that will contain the lists of vertices of the
		# space ships sides.
		self.verts = self.getWhirlybirdVertices()
		#Create stand that won't move
		stand = Line3DCollection([np.asarray(self.verts[0])],
					facecolor = 'black',edgecolor = 'black',lw=2)
		self.ax.add_collection3d(stand)
		# A list that will contain handles to the Poly3DCollection
		# Objects so that they can be modified.
		self.PolyCollections = []
		# Set axis limits
		_axis_limit = 1.1
		self.ax.set_zlim3d([-.65,_axis_limit])
		self.ax.set_ylim3d([-_axis_limit,_axis_limit])
		self.ax.set_xlim3d([-_axis_limit,_axis_limit])
		# Set title and labels
		self.ax.set_title('Whirlybird')
		self.ax.set_xlabel('East')
		self.ax.set_ylabel('North')
		self.ax.set_zlabel('-Down')
		# Change viewing angle
		self.ax.view_init(self.ax.elev, self.ax.azim+90)

		# Draw pendulum is the main function that will call the functions:
		# drawCart, drawCircle, and drawRod to create the animation.
	def drawSystem(self, u):
		# Process inputs to function
		phi = u[0]        # Horizontal position of cart, m
		theta = u[1]
		psi = u[2]   # Angle of pendulum, rads

		self.drawWhirlybird(phi,theta,psi)
		#self.ax.axis('equal') # This will cause the image to not distort

		# After each function has been called, initialization is over.
		if self.flagInit == True:
			self.flagInit = False

	def drawWhirlybird(self,phi,theta,psi):
		verts = []
		for i in range(1,len(self.verts)):
			vertsTemp = rotate(self.verts[i].T,phi,theta,psi).T
			vertsTemp = transformXYZtoNED(vertsTemp)
			verts.append(vertsTemp)

		#print(verts)
		verts = np.asarray(verts)
		#print(verts)

		if self.flagInit == True:
			# Initialize Poly3DCollection class for each set of vertices, and
			# create an object handle to each one.
			self.PolyCollections.append(Line3DCollection([np.asarray(verts[0])],facecolor = 'black',edgecolor = 'black', lw = 2))
			self.PolyCollections.append(Line3DCollection([np.asarray(verts[1])],facecolor = 'black',edgecolor = 'black', lw = 2))
			self.PolyCollections.append(Poly3DCollection([np.asarray(verts[2])],facecolor = 'green',edgecolor = 'black', lw = 2))
			self.PolyCollections.append(Poly3DCollection([np.asarray(verts[3])],facecolor = 'red',edgecolor = 'black', lw = 2))
			# Add each Poly3DCollection object to the axes.
			for i in range(len(self.PolyCollections)):
				self.ax.add_collection3d(self.PolyCollections[i])
			self.flagInit = False
		else:
			# Update the verts
			self.PolyCollections[0].set_segments([np.asarray(verts[0])])
			self.PolyCollections[1].set_segments([np.asarray(verts[1])])
			self.PolyCollections[2].set_verts([np.asarray(verts[2])])
			self.PolyCollections[3].set_verts([np.asarray(verts[3])])

	def getWhirlybirdVertices(self):
		# Whirlybird Base
		verts_base = np.matrix([ 		[0, 0, -P.h],
								 		[0, 0, 0] ])

		# Rod with counter-balance
		verts_long_rod = np.matrix([ 	[0, -P.l2, 0],
								   		[0, P.l1, 0] ])

		# Rod between rotors
		verts_short_rod = np.matrix([	[-(P.d - P.r), P.l1, 0],
										[(P.d - P.r), P.l1, 0] ])

		# West rotor (left)
		verts_rotor_left = np.matrix([	[-(P.d - P.r), P.l1 + P.r, 0],
										[-(P.d + P.r), P.l1 + P.r, 0],
										[-(P.d + P.r), P.l1 - P.r, 0],
										[-(P.d - P.r), P.l1 - P.r, 0] ])

		# East rotor (right)
		verts_rotor_right = np.matrix([	[(P.d - P.r), P.l1 + P.r, 0],
										[(P.d + P.r), P.l1 + P.r, 0],
										[(P.d + P.r), P.l1 - P.r, 0],
										[(P.d - P.r), P.l1 - P.r, 0] ])

		return [verts_base,verts_long_rod,verts_short_rod,verts_rotor_left,verts_rotor_right]

def rotate(XYZ, phi, theta, psi):
	# Define rotation matrices
	R_pitch = np.matrix([	[1, 		0,				0],
							[0,	np.cos(theta), np.sin(theta)],
							[0, -np.sin(theta), np.cos(theta)]])

	R_roll = np.matrix([	[np.cos(phi), 0,	np.sin(phi)],
							[0, 			1,				0],
							[-np.sin(phi),0,  np.cos(phi)]])

	R_yaw = np.matrix([		[np.cos(psi),  -np.sin(psi),	0],
							[np.sin(psi),   np.cos(psi),	0],
							[0, 			0,  			1]])

	# Combine roll, pitch, then yaw rotations
	R = R_yaw*R_pitch*R_roll

	#Rotate the vertices (roll -> pitch -> yaw)
	XYZ = R*XYZ

	return XYZ

def transformXYZtoNED(XYZ):
	R = np.matrix([	[0, 1, 0],
					[1, 0, 0],
					[0, 0, -1]])
	NED = XYZ*R
	return NED

# Used see the animation.
if __name__ == "__main__":
    simAnimation = Animate()
    simAnimation.drawSystem([P.zv0,P.h0,P.theta0,P.zt0])  # Draw the pendulum
    plt.show()
