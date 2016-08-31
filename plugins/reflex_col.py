from klampt import *
#Klampt v0.7
from klampt.math import se3,vectorops
from klampt.vis.glrobotprogram import *
from klampt.sim.simulation import ActuatorEmulator
#Klampt v0.6.x
#from klampt import se3,vectorops
#from klampt.glrobotprogram import *
#from klampt.simulation import ActuatorEmulator

#The hardware name
gripper_name = 'reflex'

#The Klamp't model name
klampt_model_name = 'data/robots/reflex_col.rob'

#the number of Klamp't model DOFs
numDofs = 16

#The number of command dimensions
numCommandDims = 4

#The names of the command dimensions
commandNames = ['finger1','finger2','finger3','preshape']

#default postures
openCommand = [1,1,1,0]
closeCommand = [0,0,0,0]
pinchOpenCommand = [0,0,0,1]
pinchCloseCommand = [1,1,0,1]

#named preset list
presets = {'open':openCommand,
           'closed':closeCommand,
           'pinch-open':pinchOpenCommand,
           'pinch-close':pinchCloseCommand
           }

#range of postures
commandMinimum = [0,0,0,0]
commandMaximum = [1,1,1,1]

#range of valid command velocities
commandMinimumVelocity = [-1,-1,-1,-1]
commandMaximumVelocity = [1,1,1,1]

swivel_links = [2,7]
proximal_links = [3,8,12]
distal_links = [4,9,13]

def commandToConfig(command):
    """Given a rethink parallel jaw gripper command vector, in the range
    range [0] (closed) to [1] (open), returns the gripper configuration for
    the klampt model.
    """
    global swivel_links,proximal_links,distal_links
    global numDofs
    finger1,finger2,finger3,preshape = command
    proxmin,proxmax = -0.34,2.83
    preshapemax = 1.5708
    q = [0.0]*numDofs
    q[swivel_links[0]] = preshapemax*(1-preshape)
    q[swivel_links[1]] = -preshapemax*(1-preshape)
    fingers = [finger1,finger2,finger3]
    for i in range(3):
        q[proximal_links[i]] = proxmax+fingers[i]*(proxmin-proxmax)
        q[distal_links[i]] = 0#fingers[i]*1.9
    return q

def configToCommand(config):
    """Given a gripper configuration for the klampt model, returns the
    closest command that corresponds to this configuration.  Essentially
    the inverse of commandToConfig(). 
    """
    proxmin,proxmax = -0.34,2.83
    preshapemax = 1.5708
    preshape = 1-(config[swivel_links[0]]-config[swivel_links[1]])*0.5/preshapemax
    fingers = [(config[proximal_links[i]]-proxmax)/(proxmin-proxmax) for i in range(3)]
    return fingers+[preshape]

class HandModel:
    """A kinematic model of the Reflex hand"""
    def __init__(self,robot,link_offset=0,driver_offset=0):
        """
        Arguments:
        - robot: the RobotModel instance containing the reflex_col hand.
        - link_offset: the link of the base of the hand in the robot model
        - driver_offset: the driver index of the first driver link in the robot model
        """
        global swivel_links,proximal_links,distal_links
        self.robot = robot
        self.link_offset = link_offset
        self.driver_offset = driver_offset
        qmin,qmax = self.robot.getJointLimits()
        self.preshape_driver = self.driver_offset
        self.swivel_links = [link_offset+i for i in swivel_links]
        self.proximal_links = [link_offset+i for i in proximal_links]
        self.distal_links = [link_offset+i for i in distal_links]
        self.proximal_drivers = [self.driver_offset+1,self.driver_offset+6,self.driver_offset+10]
        self.distal_drivers = [self.driver_offset+2,self.driver_offset+7,self.driver_offset+11]
        self.jointLimits = ([qmin[link_offset+proximal_links[0]],qmin[link_offset+proximal_links[1]],qmin[link_offset+proximal_links[2]],0],
                            [qmax[link_offset+proximal_links[0]],qmax[link_offset+proximal_links[1]],qmax[link_offset+proximal_links[2]],0])
    def numCommands(self):
        return 4
    def commandNames(self):
        global commandNames
        return commandNames
    def domain(self):
        #closure joints: 0=closed, 1=open
        #preshape joint 3: 0=power, 1=pinch
        return ([0,0,0,0],[1,1,1,1])
    def getCommand(self):
        preshape = self.robot.driver(self.preshape_driver).getValue()
        qrob = self.robot.getConfig()
        qmin,qmax = self.jointLimits
        fingers = [(qrob[self.proximal_links[i]]-qmax[i])/(qmin[i]-qmax[i]) for i in range(3)]
        return fingers+[preshape]
    def getVelocity(self):
        preshape = self.robot.driver(self.preshape_driver).getVelocity()
        vrob = self.robot.getVelocity()
        qmin,qmax = self.jointLimits
        fingers = [(vrob[self.proximal_links[i]])/(qmin[i]-qmax[i]) for i in range(3)]
        return fingers+[preshape]
    def setCommand(self,command):
        """Sets the configuration of self.robot given a hand config"""
        assert len(command)==4,"Reflex hand has 4 DOFS"
        [finger1,finger2,finger3,preshape] = command
        fingers = [finger1,finger2,finger3]
        self.robot.driver(self.preshape_driver).setValue(preshape)
        qrob = self.robot.getConfig()
        qmin,qmax = self.jointLimits
        for i in range(3):
            qrob[self.proximal_links[i]] = qmax[i]+fingers[i]*(qmin[i]-qmax[i])
            qrob[self.distal_links[i]] = fingers[i]*1.9
        self.robot.setConfig(qrob)
    def setVelocity(self,vel):
        """Sets the command velocity of self.robot"""
        assert len(config)==4,"Reflex hand has 4 DOFS"
        [finger1,finger2,finger3,preshape] = vel
        fingers = [finger1,finger2,finger3]
        self.robot.driver(self.preshape_driver).setVelocity(preshape)
        vrob = self.robot.getVelocity()
        qmin,qmax = self.jointLimits
        for i in range(3):
            vrob[self.proximal_links[i]] = vel[i]*(qmin[i]-qmax[i])
            vrob[self.distal_links[i]] = fingers[i]*1.9
        self.robot.setVelocity(vrob)

class HandEmulator(ActuatorEmulator):
    """A simulation model of the Reflex hand for use with SimpleSimulation
    """
    def __init__(self,sim,robotindex=0,link_offset=0,driver_offset=0):
        self.sim = sim
        world = sim.world
        self.robotindex = robotindex
        self.controller = self.sim.controller(robotindex)
        #rubber for pad
        pad = self.sim.body(world.robotLink(robotindex,link_offset+1))
        #TODO: TUNE THESE TO DESCRIBE THE FINGER PAD SURFACE
        s = pad.getSurface()
        s.kFriction = 1.5
        s.kStiffness = 20000
        s.kDamping = 20000
        pad.setCollisionPadding(0.005)
        fingerpads = [link_offset+5,link_offset+6,link_offset+10,link_offset+11,link_offset+14,link_offset+15]
        for l in fingerpads:
            pad = self.sim.body(world.robotLink(robotindex,l))
            s = pad.getSurface()
            s.kFriction = 1.5
            pad.setCollisionPadding(0.0025)
        self.world = world
        self.model = HandModel(world.robot(robotindex),link_offset,driver_offset)
        self.setpoint = self.model.getCommand()
        self.endpoint = self.setpoint[:]
        self.speed = [1,1,1,1]
        self.force = [0,0,0,0]
        self.moving = [0,0,0,0]
        self.update_tendon_lengths()
        print "Reflex Hand Simulation initialized"
        print "  Initial setpoint",self.setpoint
        print "  Rest tendon lengths:",self.tendon_lengths
        #attachment points of proximal / distal joints,
        #relative to center of mass frames
        self.base_tendon_center = [0,0,0]
        self.tendon0_local = [-0.02,0,0.009]
        self.tendon1_local = [0.035,0,0.009]
        self.tendon2_local = [-0.015,0,0.007]
        self.forces = [[None,None,None] for i in range(3)]

    def getCommand(self):
        return self.endpoint

    def setCommand(self,command):
        self.endpoint = [max(min(v,1),0) for v in command]

    def setFinger1(self,value):
        self.endpoint[0] = max(min(value,1),0)

    def setFinger2(self,value):
        self.endpoint[1] = max(min(value,1),0)

    def setFinger3(self,value):
        self.endpoint[2] = max(min(value,1),0)

    def setPreshape(self,value):
        self.endpoint[3] = max(min(value,1),0)

    def update_tendon_lengths(self):
        #drive system:
        #find deviation between commanded and actual on proximal joint, use
        #that to determine tendon lengths
        qcmd = self.controller.getCommandedConfig()
        qactual = self.sim.getActualConfig(self.robotindex)
        pulls = [qcmd[l] - qactual[l] for i,l in enumerate(self.model.proximal_links)]
        pullscale = 0.5
        self.tendon_lengths = [0,0,0]
        self.tendon_lengths[0] = max(0,1.0-pulls[0]*pullscale)*0.0215
        self.tendon_lengths[1] = max(0,1.0-pulls[1]*pullscale)*0.0215
        self.tendon_lengths[2] = max(0,1.0-pulls[2]*pullscale)*0.0215

    def process(self,commands,dt):
        if commands:
            if 'position' in commands:
                self.setCommand(commands['position'])
                del commands['position']
            if 'qcmd' in commands:
                self.setCommand(commands['qcmd'])
                del commands['qcmd']
            if 'speed' in commands:
                self.speed = commands['speed']
                del commands['speed']
            if 'force' in commands:
                self.force = commands['force']
                del commands['force']
        for i in range(4):
            speed = self.speed[i]
            if self.endpoint[i] < self.setpoint[i]:
                self.setpoint[i] = max(self.setpoint[i]-speed*dt,self.endpoint[i])
            elif self.endpoint[i] > self.setpoint[i]:
                self.setpoint[i] = min(self.setpoint[i]+speed*dt,self.endpoint[i])
        self.model.setCommand(self.setpoint)
        q = self.model.robot.getConfig()
        qcmd = self.controller.getCommandedConfig()
        qcmd[self.model.swivel_links[0]] = q[self.model.swivel_links[0]]
        qcmd[self.model.swivel_links[1]] = q[self.model.swivel_links[1]]
        qcmd[self.model.proximal_links[0]] = q[self.model.proximal_links[0]]
        qcmd[self.model.proximal_links[1]] = q[self.model.proximal_links[1]]
        qcmd[self.model.proximal_links[2]] = q[self.model.proximal_links[2]]
        qcmd[self.model.distal_links[0]] = 0
        qcmd[self.model.distal_links[1]] = 0
        qcmd[self.model.distal_links[2]] = 0
        vcmd = self.controller.getCommandedVelocity()
        #print "Hand commanded q / retrieved q:"
        #for (a,b) in zip(qcmd,self.controller.getCommandedConfig()):
        #    print "  ",a,b
        if qcmd != self.controller.getCommandedConfig():
            #allow queued movements for other joints if fingers are stopped
            self.controller.setPIDCommand(qcmd,vcmd)
        
    def substep(self,dt):
        #apply forces associated with tendon
        self.model.setCommand(self.setpoint)
        self.update_tendon_lengths()
        for i in range(3):
            self.apply_tendon_forces(i,self.model.proximal_links[i],self.model.distal_links[i],self.tendon_lengths[i])

    def apply_tendon_forces(self,i,link1,link2,rest_length):
        tendon_c2 = 30000.0
        tendon_c1 = 10000.0
        b0 = self.sim.body(self.model.robot.link(self.model.proximal_links[0]-3))
        b1 = self.sim.body(self.model.robot.link(link1))
        b2 = self.sim.body(self.model.robot.link(link2))
        p0w = se3.apply(b1.getTransform(),self.tendon0_local)
        p1w = se3.apply(b1.getTransform(),self.tendon1_local)
        p2w = se3.apply(b2.getTransform(),self.tendon2_local)

        d = vectorops.distance(p1w,p2w)
        if d > rest_length:
            #apply tendon force
            direction = vectorops.unit(vectorops.sub(p2w,p1w))
            f = tendon_c2*(d - rest_length)**2+tendon_c1*(d - rest_length)
            #print d,rest_length
            #print "Force magnitude",self.model.robot.link(link1).getName(),":",f
            f = min(f,100)
            #tendon routing force
            straight = vectorops.unit(vectorops.sub(p2w,p0w))
            pulley_direction = vectorops.unit(vectorops.sub(p1w,p0w))
            pulley_axis = so3.apply(b1.getTransform()[0],(0,1,0))
            tangential_axis = vectorops.cross(pulley_axis,pulley_direction)
            cosangle = vectorops.dot(straight,tangential_axis)
            #print "Cosine angle",self.model.robot.link(link1).getName(),cosangle
            base_direction = so3.apply(b0.getTransform()[0],[0,0,-1])
            b0.applyForceAtLocalPoint(vectorops.mul(base_direction,-f),vectorops.madd(p0w,base_direction,0.04))
            b1.applyForceAtLocalPoint(vectorops.mul(tangential_axis,cosangle*f),self.tendon1_local)
            b1.applyForceAtLocalPoint(vectorops.mul(tangential_axis,-cosangle*f),self.tendon0_local)
            b2.applyForceAtLocalPoint(vectorops.mul(direction,-f),self.tendon2_local)
            self.forces[i][1] = vectorops.mul(tangential_axis,cosangle*f)
            self.forces[i][2] = vectorops.mul(direction,f)
        else:
            self.forces[i] = [None,None,None]
        return

    def drawGL(self):
        #draw tendons
        glDisable(GL_LIGHTING)
        glDisable(GL_DEPTH_TEST)
        glLineWidth(4.0)
        glColor3f(0,1,1)
        glBegin(GL_LINES)
        for i in range(3):
            b1 = self.sim.body(self.model.robot.link(self.model.proximal_links[i]))
            b2 = self.sim.body(self.model.robot.link(self.model.distal_links[i]))
            glVertex3f(*se3.apply(b1.getTransform(),self.tendon0_local))
            glVertex3f(*se3.apply(b1.getTransform(),self.tendon1_local))
            glVertex3f(*se3.apply(b1.getTransform(),self.tendon1_local))
            glVertex3f(*se3.apply(b2.getTransform(),self.tendon2_local))
        glEnd()
        glLineWidth(1)
        glColor3f(1,0.5,0)
        glBegin(GL_LINES)
        fscale = 0.01
        for i in range(3):
            b1 = self.sim.body(self.model.robot.link(self.model.proximal_links[i]))
            b2 = self.sim.body(self.model.robot.link(self.model.distal_links[i]))
            if self.forces[i][0] != None:
                p = se3.apply(b1.getTransform(),self.tendon0_local)
                glVertex3f(*p)
                glVertex3f(*vectorops.madd(p,self.forces[i][0],fscale))
            if self.forces[i][1] != None:
                p = se3.apply(b1.getTransform(),self.tendon1_local)
                glVertex3f(*p)
                glVertex3f(*vectorops.madd(p,self.forces[i][1],fscale))
            if self.forces[i][2] != None:
                p = se3.apply(b2.getTransform(),self.tendon2_local)
                glVertex3f(*p)
                glVertex3f(*vectorops.madd(p,self.forces[i][2],fscale))
        glEnd()
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)


class HandSimGLViewer(GLSimulationProgram):
    def __init__(self,world,base_link=0,base_driver=0):
        GLSimulationProgram.__init__(self,world,"Reflex simulation program")
        self.handsim = HandEmulator(self.sim,0,base_link,base_driver)
        self.sim.addEmulator(0,self.handsim)
        self.control_dt = 0.01

    def control_loop(self):
        #external control loop
        #print "Time",self.sim.getTime()
        return

    def idle(self):
        if self.simulate:
            self.control_loop()
            self.sim.simulate(self.control_dt)
            glutPostRedisplay()

    def print_help(self):
        GLSimulationProgram.print_help()
        print "y/h: raise/lower finger 1 command"
        print "u/j: raise/lower finger 2 command"
        print "i/k: raise/lower finger 3 command"
        print "o/l: raise/lower preshape command"

    def keyboardfunc(self,c,x,y):
        #Put your keyboard handler here
        #the current example toggles simulation / movie mode
        if c=='y':
            u = self.handsim.getCommand()
            u[0] += 0.1
            self.handsim.setCommand(u)
        elif c=='h':
            u = self.handsim.getCommand()
            u[0] -= 0.1
            self.handsim.setCommand(u)
        elif c=='u':
            u = self.handsim.getCommand()
            u[1] += 0.1
            self.handsim.setCommand(u)
        elif c=='j':
            u = self.handsim.getCommand()
            u[1] -= 0.1
            self.handsim.setCommand(u)
        elif c=='i':
            u = self.handsim.getCommand()
            u[2] += 0.1
            self.handsim.setCommand(u)
        elif c=='k':
            u = self.handsim.getCommand()
            u[2] -= 0.1
            self.handsim.setCommand(u)
        elif c=='o':
            u = self.handsim.getCommand()
            u[3] += 0.1
            self.handsim.setCommand(u)
        elif c=='l':
            u = self.handsim.getCommand()
            u[3] -= 0.1
            self.handsim.setCommand(u)
        else:
            GLSimulationProgram.keyboardfunc(self,c,x,y)
        glutPostRedisplay()



        
if __name__=='__main__':
    global klampt_model_name
    world = WorldModel()
    if not world.readFile(klampt_model_name):
        print "Could not load Reflex hand from",klampt_model_name
        exit(1)
    viewer = HandSimGLViewer(world)
    viewer.run()

    
