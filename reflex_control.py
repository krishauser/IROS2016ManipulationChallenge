"""Common code for handling the Righthand Robotics ReFlex gripper"""

class ReflexController:
	"""A controller that simply provides convenient accesss to the ReFlex's contact sensors.
	It will print out contact sensor readings every type __call__ is called if verbose is set to True.
	"""
	def __init__(self,sim,hand,dt):
		self.sim = sim
		self.hand = hand
		self.dt = dt

		#get references to the robot's sensors (not properly functioning in 0.6.x)
		self.f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
		self.f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
		self.contact_sensors = self.f1_proximal_takktile_sensors + self.f1_distal_takktile_sensors + self.f2_proximal_takktile_sensors + self.f2_distal_takktile_sensors + self.f3_proximal_takktile_sensors + self.f3_distal_takktile_sensors

		self.verbose = False 

	def contact_measurements(self):
		try:
			f1_contact = [s.getMeasurements()[0] for s in self.f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in self.f1_distal_takktile_sensors]
			f2_contact = [s.getMeasurements()[0] for s in self.f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in self.f2_distal_takktile_sensors]
			f3_contact = [s.getMeasurements()[0] for s in self.f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in self.f3_distal_takktile_sensors]
			return [f1_contact,f2_contact,f3_contact]
		except:
			return None

	def __call__(self,controller):
		if self.verbose: 
			try:
				f1_contact,f2_contact,f3_contact = self.contact_measurements()
				print "Contact sensors"
				print "  finger 1:",[int(v) for v in f1_contact]
				print "  finger 2:",[int(v) for v in f2_contact]
				print "  finger 3:",[int(v) for v in f3_contact]
			except:
				pass
		#need to manually call the hand emulator
		self.hand.process({},self.dt)
