#!/usr/bin/python
import time, random, sys
from numpy import *
import scipy as Sci
from Biped_Controller import Biped_Controller
from Biped_Planning_115 import Biped_Kinematics, Biped_Planning_6DOF
import traceback

# Simple script to go to a desired point.
if len(sys.argv) != 4:
    print "Usage: x y z, target point to move right foot to"
    exit(1)
    
goal = matrix([[float(sys.argv[1])], [float(sys.argv[2])], [float(sys.argv[3])]])

test_controller = Biped_Controller()
try:
    test_planner = Biped_Planning_6DOF()
    print "Trying to find way to get to ", goal
    targ = test_planner.inverse_kin_left_leg(goal, maxattempts = 4, alpha=0.8, alphadecay=0.995, delta_eps = 0.02)
    print "Got it -- going to"
    print targ
    test_controller.go_to_pose(0.5, targ)
    print "Done!\n"
    while (True):
        pass
except:
    print "Ooops"
    traceback.print_exc()
finally:
    print "Relaxing"
    test_controller.relax()
    test_controller.__del__()
    time.sleep(1)
