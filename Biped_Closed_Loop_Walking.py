#!/usr/bin/python

from Biped_Controller import Biped_Controller
from Biped_Planning import Biped_Kinematics, Biped_Planning_6DOF
import time
import traceback
import sys
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy import optimize

# Simple script to just go to a particular point
# as derived by IK
if len(sys.argv) != 1:
    print "Usage: "
    print "\t--No args for now."
    sys.exit(0)

print "Spawning biped controller"
test_controller = Biped_Controller(debug=True)
print "Spawning planner"
test_planner = Biped_Planning_6DOF()

try:
    print "Crouching"
    next_step = { 'AR': 0, 'KR': -30, 'HR': 30,
                  'AL': 0, 'KL': -30, 'HL': 30 }
    test_controller.go_to_pose(1, next_step, False)
    time.sleep(1)

    while (True):
        print "Standing to right foot"
        next_step = { 'AR': 30, 'KR': -50, 'HR': 50,
                      'AL': -35, 'KL': 0, 'HL': 0 }
        test_controller.go_to_pose(2, next_step, False)
        time.sleep(2)
        push_pose = { 'KL': -30, 'HL': 30, 'KR': -20, 'HR': 20 }
        test_controller.balance_on_leg(['AR'], [1], 20, \
            ['KR', 'HR'], [-1, -1], 0, push_pose)
        time.sleep(2)
        
        print "Balancing, pushing left leg forward"
        curr_pose = test_controller.get_pose()
        push_pose = { 'AL': 0, 'KR': 50, 'HR': -50, 'KL':-30, 'HL': 30}
        test_controller.balance_on_leg(['AR'], [1], 20, \
            ['KR', 'HR'], [-1, -1], 0, push_pose)
        start_time = time.time()
        while (time.time() < start_time+4):
            curr_pose = test_controller.get_pose()
            push_pose = { 'AL': 0, 'KR': 50, 'HR': -50, 'KL':-40-(curr_pose['KR']-40), 'HL': 40+(-40-curr_pose['HR'])}
            test_controller.balance_on_leg(['AR'], [1], 30, \
                ['KR', 'HR'], [-1, -1], 0, push_pose)

        #print "Planning right step"
        #curr_pose = test_controller.get_pose()
        #bounds = [(curr_pose['AR']-10,curr_pose['AR']+10),(-5,5),(-60,60),(-60,60),(-45,45),(-45,45)]
        #guess = [curr_pose['AR'],0,curr_pose['KR']+50,-30,curr_pose['HR']-50,30]
        #next_step = test_planner.figure_out_delta_pose('AR', 'AL', [0.0,4.0, 0.0], curr_pose, guess, bounds=bounds)
        #test_controller.go_to_pose(3, next_step, False)
        #time.sleep(3)

        print "Standing down"
        push_pose = { 'KL': -30, 'HL': 30, 'KR': -10, 'HR': 10 }
        test_controller.balance_on_leg(['AR'], [1], -5, \
            ['KR', 'HR'], [-1, -1], -20)
        time.sleep(2)

        print "Standing to left foot"
        next_step = { 'AL': 30, 'KL': -50, 'HL': 50,
                      'AR': -35, 'KR': 0, 'HR': 0 }
        test_controller.go_to_pose(2, next_step, False)
        time.sleep(2)
        push_pose = { 'KR': -30, 'HR': 30, 'KL': -20, 'HL': 20 }
        test_controller.balance_on_leg(['AL'], [-1], -20, \
            ['KL', 'HL'], [-1, -1], 0, push_pose)
        time.sleep(2)
        
        print "Balancing, pushing right leg forward"
        push_pose = { 'AR': 0, 'KL': 50, 'HL': -50, 'KR':-30, 'HR': 30}        
        test_controller.balance_on_leg(['AL'], [-1], -20, \
            ['KL', 'HL'], [-1, -1], 0, push_pose)
        start_time = time.time()
        while (time.time() < start_time+4):
            curr_pose = test_controller.get_pose()
            push_pose = { 'AR': 0, 'KL': 50, 'HL': -50, 'KR':-40-(curr_pose['KL']-40), 'HR': 40+(-40-curr_pose['HL'])}
            test_controller.balance_on_leg(['AL'], [-1], -30, \
                ['KL', 'HL'], [-1, -1], 0, push_pose)

        #print "Planning left step"
        #curr_pose = test_controller.get_pose()
        #bounds = [(-5,5),(curr_pose['AL']-10,curr_pose['AL']+10),(-60,60),(-60,60),(-45,45),(-45,45)]
        #guess = [0,curr_pose['AL'],-30,curr_pose['KL']+50,30,curr_pose['HL']-50]
        #next_step = test_planner.figure_out_delta_pose('AL', 'AR', [0.0,4.0, 0.0], curr_pose, guess, bounds=bounds)
        #test_controller.go_to_pose(3, next_step, False)
        #time.sleep(3)
        
        print "Standing down"
        push_pose = { 'KR': -30, 'HR': 30, 'KL': -10, 'HL': 10 }
        test_controller.balance_on_leg(['AL'], [-1], 5, \
            ['KL', 'HL'], [-1, -1], -20)
        time.sleep(1)
        
    
except:
    print "Ooops"
    traceback.print_exc()
finally:
    print "Relaxing"
    test_controller.__del__()
    time.sleep(1)
        
