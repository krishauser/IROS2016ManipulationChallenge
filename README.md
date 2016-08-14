# IROS 2016 Grasping and Manipulation Competition Simulation Framework #
### Release v1.0 ###
### Kris Hauser ###
### 8/10/2016 ###

This package describes the simulation framework for the IROS 2016 Grasping and Manipulation
Challenge.  Despite its use in the IROS competition, it is also meant to be an open framework
for benchmarking of manipulation controllers.  Contributions are welcomed, particularly for
new robots, datasets, and evaluation protocols.

The package is based on the Klamp't library, using the Python 2.7 language.  Klamp't is developed
by Duke University for robotics research, visualization, and education.  Crucially for
this competition, it can perform realistic robot physics simulation with scanned 3D objects.
It can also simulate several robot sensors, such as force/torque sensors, contact sensors, and depth
cameras.


## Datasets ##

The framework currently supports the following two object datasets:
- [ycb](http://rll.eecs.berkeley.edu/ycb/): the YCB dataset.
- [apc2015](http://rll.berkeley.edu/amazon_picking_challenge/): the Amazon Picking Challenge 2015 dataset.


## Robots ##

By default, the robot is a free-floating gripper.  The gripper base may be moved arbitrarily in
(x,y,z,yaw,pitch,roll) space, while the fingers may be controlled individually.  The gripper currently
implemented is the Righthand Robotics Reflex Gripper.  It provides 4 DOF of actuation and several
contact sensors.

You can design your own robots or grippers in URDF format with Klamp't's additional XML specifications.
See the Klamp't URDF import tutorial at
http://motion.pratt.duke.edu/klampt/tutorial_import_robot.html for more details.

For more advanced sensors, actuators, and underactuation, you can also develop simulation plugins.
Documentation for this feature is still under development, but its main functionality should be fairly
straightforward.  See the Klampt/Python/klampt/sim/simulation.py file for the API, and plugins/reflex_col.py
file for an example of an implementation for the Reflex hand.


## Dependencies ##

Foremost, this requires the Klamp't 0.6.x or 0.7 Python APIs.  There are some API changes between 
the two versions, most significantly in the Python API import structure and the physics engine's sensor
simulation (specifically, the ContactSensor, LaserSensor, and DepthCameraSensor types, and functionality
for "baking in" sensors into robot files). 

Version 0.7 is strongly recommended to get the latest features and bug fixes, however, it is still
under active development and some of the older online documentation has not yet been updated.
It is possible to use 0.6.x for this competition, but you will not have the capability to simulate certain
sensors (e.g., contact sensors).

Both Klamp't 0.6.x and 0.7 have been tested on Linux Ubuntu 14.04, and should work with most versions
of Linux.  They should also work with Mac, possibly with a little work.  Windows builds for v0.7 are
not yet available.  Windows power users can build the package with some some (substantial) work and
Visual Studio.

Foremost, you will need to install the Klamp't Python API from source.  Follow the Installation 
Tutorial found on http://motion.pratt.duke.edu/klampt/tutorial_install.html.  You will need to install
Assimp and PyQt4 as instructed in the tutorial.  The master branch is still at v0.6.2, so to get
v0.7, you will need to perform the following tweaks:
- Before you build, switch to the v0.7 branch by running the following command line in the
  Klampt/ directory:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> git checkout v0.7
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  and run the following in Klampt/Library/KrisLibrary before you build the dependencies:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> git checkout plan_devel
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Once Klamp't is installed, download the YCB and APC2015 datasets as follows (note these
are rather large datasets):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> cd IROS2016ManipulationChallenge/data/objects
> python download_ycb.py
> python download_apc2015.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


## Running the framework ##

[ Note: Version 0.6.x users will need to change some imports as instructed in the comments at the
 top of test.py and plugins/reflex_col.py ]

The main test file is main.py.  A basic debugging mode is run as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> python main.py [dataset] [object index or name]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you do not specify a dataset or an object, one will be chosen for you at random.
You will see a dialog box to edit the transform of the gripper.  Once you press "OK",
you will then see a simulation pop up where the gripper hopefully picks up the object.

![Image of basic mode](https://github.com/krishauser/IROS2016ManipulationChallenge/blob/master/images/basic.png "Image of basic mode")


If you re-run the program with the same object, you will see the last gripper transform
you selected.  This is because the transforms are being saved to
IROS2016ManipulationChallenge/resources/[dataset]/*.xform.




## Running the competition tasks ##

In the competition, your job will be to develop a controller that drives the robot such that
it performs the following tasks.  Teams are allowed to use any robot model theywish, provided
that it sufficiently reproduces the behavior of some physical robot hand (either a commercial
product or an experimental device are acceptable).  Teams are also permitted to access
"omniscient" sensor data, such as the positions of moving objects and contact forces, rather
than using simulated sensors.  However, teams that restrict themselves to using simulated
sensors or use other innovative control approaches are eligible for honorable mentions.

### Programming ###

Teams will demonstrate their work on their own machines, and they may use any external modules
and data as necessary to accomplish their task.  However, the scenario setup, simulation, and
visualization code for each scenario must remain unchanged (for example, object coefficients of
friction may not be increased to make objects easier to grasp).  Adherence to these rules will be
verified by inspecting the winning team's code.

Bindings from Python to other software packages can be done through a number of means, either by 
embedding external modules into Python code or by running parallel processes and performing
inter-process communication (IPC).  Instances of the embedding approach would include building Python
wrappers or using the ctypes library to call shared library functions.  Instances of the IPC approach
would include the use of Rospy or implementing custom serial communications.

### Runs and Scoring ###

Teams will have up to 30 minutes to demonstrate the execution of these tasks, and consists of
some number of ``runs'' on a given task.  Each run continues until the team decides to stop it. 
The best scores on each task, maximized over all runs conducted on that task, will be recorded.
The task scores will then be totaled to obtain an overall score.


### Task 1 ###

Task 1 is to lift as many balls as possible from a box and deposit them into a second box. 
To run the program, you will enter:
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> python main.py balls [# of balls]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Balls that are successfully transferred receive 1 point, and balls that are dropped outside
either box incur a penalty of 0.5 points.

![Image of Task 1](https://github.com/krishauser/IROS2016ManipulationChallenge/blob/master/images/balls.png "Image of balls mode")

### Task 2 ###

Task 2 is to extract as many objects as possible from a clutered shelf.
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> python main.py shelf [# of objects]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Objects that are successfully extracted and placed into the box are given 5 points each.

![Image of Task 2](https://github.com/krishauser/IROS2016ManipulationChallenge/blob/master/images/shelf.png "Image of shelf mode")


## Bug Reporting and Contact ##

Bugs in the framework can be reported to the IROS Grasping and Manipulation Challenge mailing
list (TBD).  Bug fixes will be also reported to the mailing list.

Other questions can be directed to Kris Hauser at kris.hauser@duke.edu and Alessio Rocchi at
rocchi.alessio@gmail.com.