from klampt import *
from klampt import visualization
from klampt import resource
from klampt.simulation import *
from klampt.glrobotprogram import *
import importlib
import os
import time

#if this is True, then the object is placed inside a box
use_box = False
box_dims = (0.5,0.5,0.3)
moving_base_template_fn = 'data/robots/moving_base_template.rob'
object_template_fn = 'data/objects/object_template.obj'
objects = {}
objects['ycb'] = [f for f in os.listdir('data/objects/ycb')]
objects['apc2015'] = [f for f in os.listdir('data/objects/apc2015')]
robots = ['reflex_col']

object_geom_file_patterns = {
	'ycb':['data/objects/ycb/%s/meshes/tsdf_mesh.stl','data/objects/ycb/%s/meshes/poisson_mesh.stl'],
	'apc2015':['data/objects/apc2015/%s/textured_meshes/optimized_tsdf_textured_mesh.ply']
}
#default mass for objects whose masses are not specified, in kg
default_object_mass = 0.5
object_masses = {
	'ycb':dict(),
	'apc2015':dict(),
}
robot_files = {
	'reflex_col':'data/robots/reflex_col.rob'
}


def mkdir_p(path):
	"""Quietly makes the directories in path"""
	import os, errno
	try:
		os.makedirs(path)
	except OSError as exc: # Python >2.5
		if exc.errno == errno.EEXIST and os.path.isdir(path):
			pass
		else: raise

def make_object(object_set,objectname,world):
	"""Adds an object to the world using its geometry / mass properties
	and places it in a default location (x,y)=(0,0) and resting on plane."""
	for pattern in object_geom_file_patterns[object_set]:
		objfile = pattern%(objectname,)
		objmass = object_masses[object_set].get('mass',default_object_mass)
		f = open(object_template_fn,'r')
		pattern = ''.join(f.readlines())
		f.close()
		f2 = open("temp.obj",'w')
		f2.write(pattern % (objfile,objmass))
		f2.close()
		if world.loadElement('temp.obj') < 0 :
			continue
		obj = world.rigidObject(world.numRigidObjects()-1)
		bmin,bmax = obj.geometry().getBB()
		T = obj.getTransform()
		spacing = 0.005
		T = (T[0],vectorops.add(T[1],(-(bmin[0]+bmax[0])*0.5,-(bmin[1]+bmax[1])*0.5,-bmin[2]+spacing)))
		obj.setTransform(*T)
		obj.appearance().setColor(0.2,0.5,0.7,1.0)
		return obj
	raise RuntimeError("Unable to load object name %s from set %s"%(objectname,object_set))

def make_box(world,width,depth,height,wall_thickness=0.005,mass=float('inf')):
	"""Makes a new axis-aligned box centered at the origin with
	dimensions width x depth x height. Walls have thickness wall_thickness. 
	If mass=inf, then the box is a Terrain, otherwise it's a RigidObject
	with automatically determined inertia.
	"""
	left = Geometry3D()
	right = Geometry3D()
	front = Geometry3D()
	back = Geometry3D()
	bottom = Geometry3D()
	left.loadFile("data/objects/cube.tri")
	right.loadFile("data/objects/cube.tri")
	front.loadFile("data/objects/cube.tri")
	back.loadFile("data/objects/cube.tri")
	bottom.loadFile("data/objects/cube.tri")
	left.transform([wall_thickness,0,0,0,depth,0,0,0,height],[-width*0.5,-depth*0.5,0])
	right.transform([wall_thickness,0,0,0,depth,0,0,0,height],[width*0.5,-depth*0.5,0])
	front.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,-depth*0.5,0])
	back.transform([width,0,0,0,wall_thickness,0,0,0,height],[-width*0.5,depth*0.5,0])
	bottom.transform([width,0,0,0,depth,0,0,0,wall_thickness],[-width*0.5,-depth*0.5,0])
	#bottom.setAABB([-width*0.5,-depth*0.5,0],[width*0.5,depth*0.5,wall_thickness])
	#left.setAABB([-width*0.5,-depth*0.5,0],[-width*0.5+wall_thickness,depth*0.5,height])
	#right.setAABB([width*0.5-wall_thickness,-depth*0.5,0],[width*0.5,depth*0.5,height])
	#front.setAABB([-width*0.5,-depth*0.5,0],[width*0.5,-depth*0.5+wall_thickness,height])
	#back.setAABB([-width*0.5,depth*0.5-wall_thickness,0],[width*0.5,depth*0.5,height])
	boxgeom = Geometry3D()
	boxgeom.setGroup()
	for i,elem in enumerate([left,right,front,back,bottom]):
		g = Geometry3D(elem)
		boxgeom.setElement(i,g)
	if mass != float('inf'):
		print "Making a box a rigid object"
		bmass = Mass()
		bmass.setMass(mass)
		bmass.setCom([0,0,height*0.3])
		bmass.setInertia([width/12,depth/12,height/12])
		box = world.makeRigidObject("box")
		box.geometry().set(boxgeom)
		box.appearance().setColor(0.6,0.3,0.2,1.0)
		box.setMass(bmass)
		return box
	else:
		print "Making a box a terrain"
		box = world.makeTerrain("box")
		box.geometry().set(boxgeom)
		box.appearance().setColor(0.6,0.3,0.2,1.0)
		return box

def make_moving_base_robot(robotname,world):
	"""Converts the given fixed-base robot into a moving base robot
	and loads it into the given world.
	"""
	f = open(moving_base_template_fn,'r')
	pattern = ''.join(f.readlines())
	f.close()
	f2 = open("temp.rob",'w')
	f2.write(pattern 
		% (robot_files[robotname],robotname))
	f2.close()
	world.loadElement("temp.rob")
	return world.robot(world.numRobots()-1)

def set_moving_base_xform(robot,R,t):
	"""For a moving base robot model, set the current base rotation
	matrix R and translation t.  (Note: if you are controlling a robot
	during simulation, use send_moving_base_xform_command)
	"""
	q = robot.getConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	robot.setConfig(q)

def send_moving_base_xform_linear(controller,R,t,dt):
	"""For a moving base robot model, send a command to move to the
	rotation matrix R and translation t using linear interpolation
	over the duration dt.

	Note: with the reflex model, can't currently set hand commands
	and linear base commands simultaneously
	"""
	q = controller.getCommandedConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	controller.setLinear(q,dt)

def send_moving_base_xform_PID(controller,R,t):
	"""For a moving base robot model, send a command to move to the
	rotation matrix R and translation t by setting the PID setpoint

	Note: with the reflex model, can't currently set hand commands
	and linear base commands simultaneously
	"""
	q = controller.getCommandedConfig()
	for i in range(3):
		q[i] = t[i]
	roll,pitch,yaw = so3.rpy(R)
	q[3]=yaw
	q[4]=pitch
	q[5]=roll
	v = controller.getCommandedVelocity()
	controller.setPIDCommand(q,v)

def launch_simple(robotname,object_set,objectname):
	world = WorldModel()
	world.loadElement("data/terrains/plane.env")
	robot = make_moving_base_robot(robotname,world)
	object = make_object(object_set,objectname,world)
	if use_box:
		box = make_box(world,*box_dims)
		object.setTransform(*se3.mul((so3.identity(),[0,0,0.01]),object.getTransform()))
	doedit = True
	xform = resource.get("%s/default_initial_%s.xform"%(object_set,robotname),description="Initial hand transform",default=robot.link(5).getTransform(),world=world)
	set_moving_base_xform(robot,xform[0],xform[1])
	xform = resource.get("%s/initial_%s_%s.xform"%(object_set,robotname,objectname),description="Initial hand transform",default=robot.link(5).getTransform(),world=world,doedit=False)
	if xform:
		set_moving_base_xform(robot,xform[0],xform[1])
	xform = resource.get("%s/initial_%s_%s.xform"%(object_set,robotname,objectname),description="Initial hand transform",default=robot.link(5).getTransform(),world=world,doedit=doedit)
	if not xform:
		print "User quit the program"
		return
	#this sets the initial condition for the simulation
	set_moving_base_xform(robot,xform[0],xform[1])

	#now the simulation is launched
	program = GLSimulationProgram(world)
	sim = program.sim

	#setup some simulation parameters
	visPreshrink = True
	for l in range(robot.numLinks()):
		sim.body(robot.link(l)).setCollisionPreshrink(visPreshrink)
	for l in range(world.numRigidObjects()):
		sim.body(world.rigidObject(l)).setCollisionPreshrink(visPreshrink)

	#get references to the robot's sensors
	f1_proximal_takktile_sensors = [sim.controller(0).sensor("f1_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f1_distal_takktile_sensors = [sim.controller(0).sensor("f1_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_proximal_takktile_sensors = [sim.controller(0).sensor("f2_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f2_distal_takktile_sensors = [sim.controller(0).sensor("f2_distal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_proximal_takktile_sensors = [sim.controller(0).sensor("f3_proximal_takktile_%d"%(i,)) for i in range(1,6)]
	f3_distal_takktile_sensors = [sim.controller(0).sensor("f3_distal_takktile_%d"%(i,)) for i in range(1,6)]
	contact_sensors = f1_proximal_takktile_sensors + f1_distal_takktile_sensors + f2_proximal_takktile_sensors + f2_distal_takktile_sensors + f3_proximal_takktile_sensors + f3_distal_takktile_sensors

	#create a hand emulator from the given robot name
	module = importlib.import_module('plugins.'+robotname)
	#emulator takes the robot index (0), start link index (6), and start driver index (6)
	hand = module.HandEmulator(sim,0,6,6)
	sim.addEmulator(0,hand)

	#the controller sends a command to the hand: f1,f2,f3,preshape
	hand.setCommand([0.2,0.2,0.2,0])

	#By running this program you get a little more control over the simulation run, but it is
	#incompatible with the Qt resource editor...
	#program.run()
	#start the simulation
	visualization.add("world",world)
	visualization.show()
	t0 = time.time()
	while visualization.shown():
		try:
			f1_contact = [s.getMeasurements()[0] for s in f1_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f1_distal_takktile_sensors]
			f2_contact = [s.getMeasurements()[0] for s in f2_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f2_distal_takktile_sensors]
			f3_contact = [s.getMeasurements()[0] for s in f3_proximal_takktile_sensors] + [s.getMeasurements()[0] for s in f3_distal_takktile_sensors]
			print "Contact sensors"
			print "  finger 1:",[int(v) for v in f1_contact]
			print "  finger 2:",[int(v) for v in f2_contact]
			print "  finger 3:",[int(v) for v in f3_contact]
		except:
			pass
		if sim.getTime() > 1:
			#the controller sends a command to the base after 1 s to lift the object
			desired = se3.mul((so3.identity(),[0,0,0.10]),xform)
			send_moving_base_xform_linear(sim.controller(0),desired[0],desired[1],0.5)
		visualization.lock()
		sim.simulate(0.01)
		sim.updateWorld()
		visualization.unlock()
		t1 = time.time()
		time.sleep(max(0.01-(t1-t0),0.001))
		t0 = t1

import random
try:
	dataset = sys.argv[1]
except IndexError:
	dataset = random.choice(objects.keys())
try:
	index = int(sys.argv[2])
except IndexError:
	index = random.randint(0,len(objects[dataset])-1)
launch_simple("reflex_col",dataset,objects[dataset][index])
visualization.kill()
