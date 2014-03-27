import math
import matplotlib.pyplot
import numpy
import sys

class GripperModel():
	# ALL COORDINATES ARE BASED ON THEIR LINKAGE'S LOCAL REFERENCE FRAME
	# ALL COORDS IN MILLIMETRES
	# pin positions
	
	print_debug = False
	
	p0x = -25.0
	p0y = 2.0
	p0radius = 4.0

	p1x = -45.0
	p1y = 4.0

	p2x = -8.4
	p2y = 2.3

	p3x = -44.0
	p3y = -5.0
	# joint positions on their linkage
	j0x = -35.0
	j0y = 0.0
	
	approxTj0_x = -23.71
	approxTj0_y = 3.528 # PRECALCULATED
	
	restT0x = None
	restT0y = None 

	j1x = -60.0
	j1y = 0.0
	j1radius = 3.2 # 3.2mm radius
	# link lengths
	l0 = 35.0 # palm
	l1 = 60.0 # proximal
	l2 = 50.0 # distal 
	
	thickness = 6.0 # mm. link thickness - distance from joint axis to gripping face.
	# arbitrary cable tension - used as a placeholder.
	cableTension = 10.0 # Newtons
	
	theta1min = 20.0;
	theta1max = 180.0 - math.degrees(math.acos( (abs(j0x)/2.0 - thickness) / l1 )) 
	theta_j1_transition = 43.5 # Angle (deg) at which the tendon starts wrapping around the J1 joint - changes the motion
	tendon_length_open = 38.90 # mm
	tendon_length_closed = 30.04 # mm 
	
	def __init__(self, print_debug_on):
		self.print_debug = print_debug_on
	
	def calculateGripperPosition(self, desiredTheta):
		# joint angles
		self.theta1 = -desiredTheta
		self.theta2 = 90.0 + self.theta1

		print "\nCALCULATING GRIPPER PARAMETERS: theta1 %.02f deg" % self.theta1
		if self.print_debug:			
			print "worldJ0: \tx: %.02f, \ty: %.02f" % (self.j0x, self.j0y)
			print "worldP0: \tx: %.02f, \ty: %.02f" % (self.p0x, self.p0y)
		# Calculate the position of P1 in the world frame (assumed to the palm's coordinate frame)
		self.relP1x = (-self.l1-self.p1x)*math.cos(math.radians(self.theta1)) - (self.p1y)*math.sin(math.radians(self.theta1))
		self.relP1y = (-self.l1-self.p1x)*math.sin(math.radians(self.theta1)) + (self.p1y)*math.cos(math.radians(self.theta1))
	
		self.worldP1x = self.relP1x + self.j0x
		self.worldP1y = self.relP1y + self.j0y
		
		if self.print_debug:
			print "worldP1 \tx: %.02f, \ty: %.02f" % (self.worldP1x, self.worldP1y)
	
		# Calculate the pickup position of the tendon on the pulley for P0
		R = self.p0radius
		x0 = self.worldP1x - self.p0x
		y0 = self.worldP1y - self.p0y
		
		# coords relative to the pulley
		self.T0x_a = ((R**2)*x0 + R*y0*math.sqrt(x0**2 + y0**2 - R**2))/(x0**2 + y0**2)
		self.T0x_b = ((R**2)*x0 - R*y0*math.sqrt(x0**2 + y0**2 - R**2))/(x0**2 + y0**2)
		self.T0y_a = (R**2 - self.T0x_a*x0)/y0
		self.T0y_b = (R**2 - self.T0x_b*x0)/y0
		
		#print "Calculating tendon pickup on P0 pulley"
		#print "T0_a x: %s, y: %s" % (self.T0x_a, self.T0y_a)
		#print "T0_b x: %s, y: %s" % (self.T0x_b, self.T0y_b)
		#print "Using T0_a\n"
		
		self.relT0_x = self.T0x_a
		self.relT0_y = self.T0y_a

		if self.theta1 == -self.theta1min:
			self.restT0x = self.relT0_x
			self.restT0y = self.relT0_y
	
	
		# convert tangential tendon pickup to world coords.
		self.worldT0x = self.p0x + self.relT0_x
		self.worldT0y = self.p0y + self.relT0_y
		
		if self.print_debug:
			print "worldT0 \tx: %.02f, \ty %.02f" % (self.worldT0x, self.worldT0y)
	
	
		# Calculate the torque in joint 1
		# Vectors showing the direction of force (p1 to the pickup on the p0 pulley), and the vector from P1 to the pivot point J0
		forceDir_x = self.worldT0x - self.worldP1x
		forceDir_y = self.worldT0y - self.worldP1y
	
		#print "forceDir_x: %s\nforceDir_y: %s" % (forceDir_x, forceDir_y)
	
		j0p1Dir_x = self.j0x - self.worldP1x
		j0p1Dir_y = self.j0y - self.worldP1y
	
		#print "j0p1Dir_x: %s\nj0p1Dir_y: %s" % (j0p1Dir_x, j0p1Dir_y)
	
		magForceDir = math.sqrt(forceDir_x**2 + forceDir_y**2)
		magj0p1Dir = math.sqrt(j0p1Dir_x**2 + j0p1Dir_y**2)
	
		#print "magForceDir: %s" % magForceDir
		#print "magj0p1Dir: %s" % magj0p1Dir	
	
		gamma = math.acos( (forceDir_x*j0p1Dir_x + forceDir_y*j0p1Dir_y) / (math.sqrt(j0p1Dir_x**2+j0p1Dir_y**2)*math.sqrt(forceDir_x**2+forceDir_y**2)) )
	
		#print "gamma (deg): %s" % math.degrees(gamma)
	
		# Calculate effective moment arm for the force.
		effectiveLength = magj0p1Dir * math.sin(gamma)
		Tj0 = self.cableTension * effectiveLength/1000 # units for all measurements are mm not m
	
		#print "effectiveLength: %s" % effectiveLength
		#print "torque j0: %s\n" % Tj0
	
	
	
		# CALCULATE JOINT 1 position
		self.relJ1_x = (self.j1x)*math.cos(math.radians(self.theta1)) - (self.j1y)*math.sin(math.radians(self.theta1))
		self.relJ1_y = (self.j1x)*math.sin(math.radians(self.theta1)) + (self.j1y)*math.cos(math.radians(self.theta1))
		if self.print_debug:
			print "relJ1_x: %s, relJ1_y: %s" % (self.relJ1_x, self.relJ1_y)
	
		self.worldJ1_x = self.relJ1_x + self.j0x
		self.worldJ1_y = self.relJ1_y + self.j0y
		if self.print_debug:
			print "worldJ1 \tx: %.02f, \ty: %.02f" % (self.worldJ1_x, self.worldJ1_y)
	
		# P2 tendon pickup point
		self.relP2_x = (self.j1x-self.p2x)*math.cos(math.radians(self.theta1)) - (self.p2y)*math.sin(math.radians(self.theta1))
		self.relP2_y = (self.j1x-self.p2x)*math.sin(math.radians(self.theta1)) + (self.p2y)*math.cos(math.radians(self.theta1))
		
		self.worldP2_x = self.relP2_x + self.j0x
		self.worldP2_y = self.relP2_y + self.j0y
		
		if self.print_debug:
			print "worldP2 \tx: %.02f, \ty: %.02f" % (self.worldP2_x, self.worldP2_y)
			print "relP2 \tx: %.02f, \ty: %.02f, \tlength: %.02f" % (self.worldP2_x-self.worldJ1_x, self.worldP2_y-self.worldJ1_y, math.sqrt((self.worldP2_x-self.worldJ1_x)**2 + (self.worldP2_y-self.worldJ1_y)**2))
		# P3 tendon pickup point
		self.relP3_x = -(self.l2+self.p3x)*math.cos(math.radians(-90.0)) - -(0.0-self.p3y)*math.sin(math.radians(-90.0))
		self.relP3_y = -(self.l2+self.p3x)*math.sin(math.radians(-90.0)) + -(0.0-self.p3y)*math.cos(math.radians(-90.0))
	
		self.worldP3_x = self.relP3_x + self.worldJ1_x
		self.worldP3_y = self.relP3_y + self.worldJ1_y
		if self.print_debug:
			print "relP3 \tx: %.02f, \ty: %.02f" % (self.relP3_x, self.relP3_y)
			print "worldP3 \tx: %.02f, \ty: %.02f" % (self.worldP3_x, self.worldP3_y)
	
		# Calculate pickup point on J1 pulley from P2
		R = self.j1radius
		x0 = self.worldP2_x - self.worldJ1_x
		y0 = self.worldP2_y - self.worldJ1_y
	
		# coords relative to the radius
		self.Tp2x_a = ((R**2)*x0 + R*y0*math.sqrt(x0**2 + y0**2 - R**2))/(x0**2 + y0**2)
		self.Tp2x_b = ((R**2)*x0 - R*y0*math.sqrt(x0**2 + y0**2 - R**2))/(x0**2 + y0**2)
		self.Tp2y_a = (R**2 - self.Tp2x_a*x0)/y0
		self.Tp2y_b = (R**2 - self.Tp2x_b*x0)/y0
	
		# Calculate pickup point on J1 pulley from P3
		self.relTp2_x = self.Tp2x_b
		self.relTp2_y = self.Tp2y_b
		
		if self.print_debug:
			print "relTp2 \tx: %.02f, \ty: %.02f" % (self.relTp2_x, self.relTp2_y)
			print "Tp2 a: (%s, %s);  b: (%s, %s)" % (self.Tp2x_a, self.Tp2y_a, self.Tp2x_b, self.Tp2y_b)
#		if (self.Tp2x_b >= 0):
#			self.relTp2_y = self.Tp2y_b
#			self.relTp2_x = self.Tp2x_b
	
		self.worldTp2_x = self.relTp2_x + self.worldJ1_x
		self.worldTp2_y = self.relTp2_y + self.worldJ1_y
		
		if self.print_debug:
			print "worldTp2 \tx: %.02f, \ty: %.02f" % (self.worldTp2_x, self.worldTp2_y)
	
		# Calculate pickup point on J1 pulley from P3
		R = self.j1radius
		x0 = self.worldP3_x - self.worldJ1_x
		y0 = self.worldP3_y - self.worldJ1_y
	
		# coords relative to the radius
		self.Tp3x_a = ((R**2)*x0 + R*y0*math.sqrt(x0**2 + y0**2 - R**2))/(x0**2 + y0**2)
		self.Tp3x_b = ((R**2)*x0 - R*y0*math.sqrt(x0**2 + y0**2 - R**2))/(x0**2 + y0**2)
		self.Tp3y_a = (R**2 - self.Tp3x_a*x0)/y0
		self.Tp3y_b = (R**2 - self.Tp3x_b*x0)/y0
	
		# Calculate pickup point on J1 pulley from P3
		self.relTp3_x = self.Tp3x_a
		self.relTp3_y = self.Tp3y_a
		
		if self.print_debug:
			print "relTp3 \tx: %.03f, \ty: %.03f" % (self.relTp3_x, self.relTp3_y)
			print "Tp3 a: (%s, %s);  b: (%s, %s)" % (self.Tp3x_a, self.Tp3y_a, self.Tp3x_b, self.Tp3y_b)
#		if (self.Tp3y_b >= 0):
#	        	self.relTp3_y = self.Tp3y_b
#		        self.relTp3_x = self.Tp3x_b
#		        print "using tp3b"
	
		
	
		self.worldTp3_x = self.relTp3_x + self.worldJ1_x
		self.worldTp3_y = self.relTp3_y + self.worldJ1_y	
		
		if self.print_debug:
			print "worldTp3 \tx: %.02f, \ty: %.02f" % (self.worldTp3_x, self.worldTp3_y)
	
	
		# Calculate arc length between two pickups.
		# Calculate the angle between the points (relative coordinates wrt the J1 joint pos)
		self.psi = math.acos( (self.relTp2_x*self.relTp3_x + self.relTp2_y*self.relTp3_y)/(R**2) )
		
		if self.print_debug:
			print "psi: %s" % math.degrees(self.psi)
	
		# length of tendon for this angle:
		self.J1_arc_length = self.psi/(2*math.pi) * 2*math.pi*self.j1radius # arc length calculation for this angle. The cancelled terms are left for clarity
	
		#print "J1 arc: %s" % self.J1_arc_length


	def getTendonLength(self):
		# RETURNS JOINT 1 length
		self.tl0 = math.sqrt((self.worldT0x-self.worldP1x)**2 + (self.worldT0y-self.worldP1y)**2)
		
		angleP0 = math.acos( (self.relT0_x*self.restT0x + self.relT0_y*self.restT0y) / (self.p0radius**2) )
		arc = self.p0radius*angleP0
		#print "ARC: %.02f" % arc
		self.tl0 = self.tl0 - arc 
		
		if self.print_debug:
			print "\nJoint 0 length (using full model):"
			print "L0: %.02f" % self.tl0
		
		#print "Joint 0 length:"
		#print "Approximate: (assumed fixed point)
		
		
		#print "Actual:"
		
		#print "Error: "
		
		if self.print_debug:
			print "\nJoint 1 length (using full model):"
		if self.worldTp2_y > self.worldTp3_y: # If tendon tangents are crossed - the tendon sits off the joint
			self.tlj1 = math.sqrt((self.worldP2_x - self.worldP3_x)**2 + (self.worldP3_y-self.worldP2_y)**2)
			a = math.sqrt( (self.worldP2_x-self.worldJ1_x)**2 + (self.worldP2_y-self.worldJ1_y)**2 )
			b = math.sqrt( (self.worldP3_x-self.worldJ1_x)**2 + (self.worldP3_y-self.worldJ1_y)**2 )
			
			psi = math.acos( (a**2 + b**2 - self.tlj1**2) / (2*a*b) ) 
			#print "a: %.02f, b: %.02f" % (a,b)
			if self.print_debug:
				print "Lj1: %.02f" % self.tlj1
				#print "psi: %.02f\n" % math.degrees(psi)
				print "\nTotal tendon length (ignoring static links): %.02f\n" % (self.tlj1 + self.tl0)
			return self.tlj1
		else:
			self.tl3 = math.sqrt((self.worldTp3_x-self.worldP3_x)**2 + (self.worldTp3_y-self.worldP3_y)**2)	
			self.tl2 = self.J1_arc_length
			self.tl1 = math.sqrt((self.worldTp2_x-self.worldP2_x)**2 + (self.worldTp2_y-self.worldP2_y)**2)
			if self.print_debug:
				print "L3: %.02f" % self.tl3
				print "L2: %.02f" % self.tl2	
				print "L1: %.02f" % self.tl1
				print "L J1 total: %.02f \n" % (self.tl3 + self.tl2 + self.tl1) 
				print "\nTotal tendon length (ignoring static links): %.02f\n" % (self.tl3 + self.tl2 + self.tl1 + self.tl0)
			return self.tl3 + self.tl2 + self.tl1

	def getEffectiveDistance(self):
		x2 = self.worldT0x
		y2 = self.worldT0y
		x1 = self.worldP1x
		y1 = self.worldP1y
		x0 = self.j0x
		y0 = self.j0y
		
		line_length = math.sqrt( (x2-x1)**2 + (y2-y1)**2 )
		effective_distance = abs( (x2-x1)*(y1-y0) - (y2-y1)*(x1-x0) ) / line_length
		#print "effective_distance : %.02f" % effective_distance
		
		return effective_distance
	
	def getJ0Torque(self, tendon_force):
		effective_distance = self.getEffectiveDistance()
		torque = tendon_force * effective_distance/1000 # Nm
		return torque
	
	def getGripperForce(self, torque):
		f_perpendicular = torque/(self.l1/1000)
		force_gripper = f_perpendicular*math.cos(math.radians(self.theta2))#abs(torque / (self.worldJ1_y/1000))
		return force_gripper

	def getGapWidth(self):
		gapWidth = -2.0*(self.worldJ1_x + self.thickness + (-self.j0x/2)) # Centre of palm, and all coords are negative
		if self.print_debug:
			print "Gap width: %.02f mm" % gapWidth
		return gapWidth


	def getAngleFromLength(self, tendonLength):
		# NB: tendonLength is the tendon extension from rest, NOT the complete tendon length (as this makes assumptions based on calibration) 
		
		# Must have a 
		baseTendonLength =  5

	def getLengthSetpointJ0(self, desiredTheta):
		# Given a desired angle theta1 -> return the required tendon length adjustment for J0.
		desiredTheta1 = desiredTheta
		# relative coords are all referenced against J0.
		a = math.sqrt((self.approxTj0_x-self.j0x)**2 + (self.approxTj0_y-self.j0y)**2)
		b = math.sqrt(self.relP1x**2 + self.relP1y**2)
		
		
		# Calculate the desired psi angle formed by this triangle - relates to theta1, and the tendon pickup points
		alpha = math.atan(self.p1y/(self.p1x+self.l1))
		if self.print_debug:
			print "alpha: %.02f" % math.degrees(alpha)
		
		beta = math.atan((self.approxTj0_y)/(self.approxTj0_x-self.j0x))
		if self.print_debug:
			print "beta: %.02f" % math.degrees(beta)
		
		# Psi (angle between P2 and P3 with J1) is defined by:
		desiredPsi = math.pi - math.radians(desiredTheta1) - beta - alpha #theta1+alpha+beta+psi = 180 degrees.
		if self.print_debug:
			print "psi: %.02f" % math.degrees(desiredPsi)
		
		desiredL = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(desiredPsi))
		print "desiredL J0: %.02f" % desiredL
		return desiredL


	def getLengthSetpointJ1(self, desiredTheta):
		# Given a desired angle theta1 (relating to gap size), what's the required tendon length.
		desiredTheta2 = 90.0-desiredTheta
		desiredTheta1 = desiredTheta
		
		deltaL = 0.0
		if desiredTheta1 > self.theta_j1_transition:		
			cappedTheta1 = self.theta_j1_transition
			diffTheta1 = desiredTheta1 - cappedTheta1
			deltaL = math.radians(diffTheta1)*self.j1radius
		else:
			cappedTheta1 = desiredTheta1
		
		
		
		# relative coords are all referenced against J1.
		a = math.sqrt((self.p3x+self.l2)**2 + (self.p3y)**2)
		b = math.sqrt(self.p2x**2 + self.p2y**2)
		
		beta = math.atan(abs(self.p2y/self.p2x))
		if self.print_debug:
			print "beta: %.02f" % math.degrees(beta)
		
		alpha = math.atan(abs(self.p3y/(self.p3x + self.l2)))
		if self.print_debug:
			print "alpha: %.02f" % math.degrees(alpha)
		
		# Psi (angle between P2 and P3 with J1) is defined by:
		desiredPsi = math.pi/2.0+math.radians(cappedTheta1)+alpha-beta
		if self.print_debug:
			print "psi: %.02f" % math.degrees(desiredPsi)
		
		desiredL = math.sqrt(a**2 + b**2 - 2*a*b*math.cos(desiredPsi))
		print "desired straight L J1: %.02f" % desiredL
		
		desiredL = desiredL + deltaL
		print "totalL J1: %.02f" % desiredL
			
		
		return desiredL
		
		

# CALCULATE THE SIMPLIFIED RELATIONSHIP BETWEEN TENDON LENGTH AND THETA1
if __name__ == "__main__":
	theta_log = []
	l_j0_log = []
	l_j1_log = []
	l_total_log = []
	gap_log = []
	force_log = []
	effective_dist_log = []
	torque_log = []
	
	gripper = GripperModel(print_debug_on=False)
	
	l_j0_rest = 0.0
	l_j1_rest = 0.0
	gap_rest = 0.0
	
	for i in range(gripper.theta1min, gripper.theta1max+1):
		gripper.calculateGripperPosition(i)
			
		l_j1 = gripper.getTendonLength()
		l_j0 = gripper.tl0
		gap = gripper.getGapWidth()
		
		effective_distance = gripper.getEffectiveDistance()
		torque = gripper.getJ0Torque(150.0)
		force = gripper.getGripperForce(torque) # 20 N applied tension
		
		print "LENGTH J0: %.02f J1: %.02f" % (l_j0, l_j1)
		print "GAP: %.02f" % (gap)
		if i == gripper.theta1min:
			l_j0_rest = l_j0
			l_j1_rest = l_j1 
			gap_rest = gap
		
		theta_log.append(i)
		l_j0_log.append((l_j0 - l_j0_rest)/1000.0)
		l_j1_log.append((l_j1 - l_j1_rest)/1000.0)
		l_total_rest = (l_j0_rest + l_j1_rest)
		l_total_log.append((l_j0 + l_j1 - l_total_rest)/1000.0)
		gap_log.append(gap/1000.0)
		effective_dist_log.append(effective_distance/1000.0)
		torque_log.append(torque)
		force_log.append(force)
		
	
	angleP0 = math.acos( (gripper.relT0_x*gripper.restT0x + gripper.relT0_y*gripper.restT0y) / (gripper.p0radius**2) )
	arc = gripper.p0radius*angleP0 
	
	print "\n**********\nRest P0 tendon point: %.02f, %.02f" % (gripper.restT0x, gripper.restT0y)
	print "Angle: %.02f" % math.degrees(angleP0)
	print "Arc: %.02f" % arc
	print "Closed P0 tendon point: %.02f, %.02f\n**********\n" % (gripper.relT0_x, gripper.relT0_y)

	print "Theta: \topen %.07f, \tmid %.07f, \tclosed %.07f" % (theta_log[0], theta_log[40], theta_log[80])
	print "Ltot: \topen %.07f, \tmid %.07f, \tclosed %.07f" % (l_total_log[0], l_total_log[40], l_total_log[80])
	print "Gap: \topen %.07f, \tmid %.07f, \tclosed %.07f" % (gap_log[0], gap_log[40], gap_log[80])
	print "ED: \topen %.07f, \tmid %.07f, \tclosed %.07f" % (effective_dist_log[0], effective_dist_log[40], effective_dist_log[80])
	print "Force: \topen %.07f, \tmid %.07f, \tclosed %.07f" % (force_log[0], force_log[40], force_log[80])

	x = gap_log
	y = force_log
	
	matplotlib.pyplot.figure()
	
	filepath = None
	if len(sys.argv) > 1:
		filepath = sys.argv[1]
	if filepath:
		filelines = file(filepath).readlines()

		graspit_x = []
		graspit_y = []
	
	
		for line in filelines:
			floats = [float(number) for number in line.split(' ')]
			if len(floats) == 2:
				graspit_x.append(math.degrees(floats[1]))
				graspit_y.append(floats[0])	
	
		matplotlib.pyplot.plot(graspit_x, graspit_y, 'g')
	
	matplotlib.pyplot.plot(x, y, 'b')
	
	coeffs = numpy.lib.polyfit(x, y, 4) 
	fit_y = numpy.lib.polyval(coeffs, x) 
	matplotlib.pyplot.plot(x, fit_y, 'r')
	
	print coeffs
	
	matplotlib.pyplot.show()
