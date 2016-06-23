# IROS 2016 Grasping and Manipulation Challenge Simulation Framework #
### Draft version 0.1 ###
### Kris Hauser ###
### 6/23/2016 ###

This package describes the simulation framework for the IROS 2016 Grasping and Manipulation
Challenge. It is meant to be an open framework and I welcome contributions, particularly for
new robots, datasets, and evaluation protocols.


## Datasets ##

The framework currently supports the following two datasets
- ycb: the YCB dataset.
- apc2015: the Amazon Picking Challenge 2015 dataset.


## Robots ##

The one gripper currently implemented is the Righthand Robotics Reflex Gripper.  

You can design your own hands in URDF format with Klamp't's additional XML specifications.
See the Klamp't URDF import tutorial for more details.

For more advanced sensors, actuators, and underactuation, you can also develop simulation plugins.
See the plugins/reflex_col.py file for more detail.


## Dependencies ##

This package has been tested on Linux Ubuntu 14.04 but should work with most versions of Linux.
It should also work with Mac, possibly with a little work.

Foremost, you will need to install the Klamp't Python API from source.  Follow the instructions on
http://klampt.org, except that once you git clone the Klampt project, you will first need to
switch to the kh_simdevel git branch as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> ...
> cd Klampt
> git checkout kh_simdevel
> ...
> [continue with the rest of the tutorial]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that you will also need to install Assimp and PyQt4.

Once Klamp't is installed, download the YCB and APC2015 datasets as follows (note these
are rather large datasets):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> cd IROS2016ManipulationChallenge/data/objects
> python download_ycb.py
> python download_apc2015.py
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


## Running the framework ##

The main test file is main.py.  It is run as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
> python main.py [dataset] [index]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you do not specify a dataset or an index, one will be chosen for you at random.
You will see a dialog box to edit the transform of the gripper.  Once you press "OK",
you will then see a simulation pop up where the gripper hopefully picks up the object.

If you re-run the program with the same index, you will see the last gripper transform
you selected.  This is because the transforms are being saved to
IROS2016ManipulationChallenge/resources/[dataset]/*.xform.



