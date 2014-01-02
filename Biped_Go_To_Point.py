#!/usr/bin/python

from Biped_Controller import Biped_Controller
from Biped_Planning import Biped_Kinematics
import time
import traceback
import sys
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy import optimize

def objective_func_sixdof(angs, targ_al, test_planner):
    ''' Messy, temporary tests of optimization library. '''
    test_angles = {
        'AR' : angs[0],
        'AL' : angs[1],
        'KR' : angs[2],
        'KL' : angs[3],
        'HR' : angs[4],
        'HL' : angs[5]
    }
    out = test_planner.get_forward_kinematics_deg('AR', test_angles)
    err = linalg.norm(out['AL']-targ_al)
    #emphasize sparsity on the hip joints
    err += 0.01*abs(angs[4]) + 0.01*abs(angs[5])
    #and add in heavy penalty for clipping ground
    diff = abs(linalg.norm(cross(transpose(out['KL']-out['AL']), matrix([[0,0,1]]))))
    err += 0.1*diff

    return err

def ground_constraint_l(angs, targ_al, test_planner):
    ''' Returns 0 when all joints above ground.'''
    test_angles = {
        'AR' : angs[0],
        'AL' : angs[1],
        'KR' : angs[2],
        'KL' : angs[3],
        'HR' : angs[4],
        'HL' : angs[5]
    }
    out = test_planner.get_forward_kinematics_deg('AR', test_angles)
    # Depending on the angle of the foot, the ground clearance
    # requirement differs.
    diff = abs(linalg.norm(cross(transpose(out['KL']-out['AL']), matrix([[0,0,1]]))))
    return float(out['AL'][2])-diff

# Simple script to just go to a particular point
# as derived by IK
if len(sys.argv) != 4:
    print "Usage: "
    print "\t--X to place left foot at"
    print "\t--Y to place left foot at"
    print "\t--Z to place left foot at"
    print "\t--A reasonable starting place: [-2.9, 1.8, 1.0]"
    sys.exit(0)

x = float(sys.argv[1])
y = float(sys.argv[2])
z = float(sys.argv[3])

print "Spawning biped controller"
test_controller = Biped_Controller(debug=True)

try:
    print "Crouching"
    next_step = { 'AR': 0, 'KR': -30, 'HR': 30,
                  'AL': 0, 'KL': -30, 'HL': 30 }
    test_controller.go_to_pose(1, next_step, False)
    print "Standing to right foot"
    next_step = { 'AR': 35, 'KR': -50, 'HR': 50,
                  'AL': -35, 'KL': 0, 'HL': 0 }
    test_controller.go_to_pose(3, next_step, False)

    print "Spawning kinematics object"
    test_planner = Biped_Kinematics(debug=True)

    print "Trying a minimization"
    targ_al = matrix([[x],[y],[z]])
    print "Target: ", targ_al
    res = optimize.fmin_slsqp(objective_func_sixdof, 
        [0.,0.,0.,0.,0.,0.],#[35.,-35.,-50.,0.,50.,0.], 
        ieqcons = [ground_constraint_l],
        args=(targ_al, test_planner),
        bounds=[(-45,45),(-45,45),(-60,60),(-60,60),(-45,45),(-45,45)],
        iter=1000000,
        iprint=1,
        acc=0.0001,
        epsilon=0.15)
    print "Results:", res

    print "\n\n generates:"
    test_angles = {
        'AR' : res[0],
        'AL' : res[1],
        'KR' : res[2],
        'KL' : res[3],
        'HR' : res[4],
        'HL' : res[5]
    }
    out = test_planner.get_forward_kinematics_deg('AR', test_angles)
    print "AL: ", out['AL']

    print "Going to...:"
    test_angles = {
        'AR' : res[0],
        'AL' : res[1],
        'KR' : res[2],
        'KL' : res[3],
        'HR' : res[4],
        'HL' : res[5]
    }
    print test_angles
    test_controller.go_to_pose(5, test_angles, False)

    steps = []
    numsteps = 24
    for i in xrange(0, numsteps, 1):
        t = i*2.*pi/numsteps
        last = [test_angles['AR'], test_angles['AL'],
                test_angles['KR'], test_angles['KL'],
                test_angles['HR'], test_angles['HL']]
        targ_al_diff = targ_al + matrix([[0.0],[2.*sin(t)],[0.0]])
        print "\n\nTarget: ", targ_al_diff
        res = optimize.fmin_slsqp(objective_func_sixdof, 
            last,
            ieqcons = [ground_constraint_l],
            args=(targ_al_diff, test_planner),
            bounds=[(-45,45),(-45,45),(-60,60),(-60,60),(-45,45),(-45,45)],
            iter=1000,
            iprint=0,
            acc=0.0001,
            epsilon=0.10)        
        #print "Results:", res

        #print "\n\n generates:"
        test_angles = {
            'AR' : res[0],
            'AL' : res[1],
            'KR' : res[2],
            'KL' : res[3],
            'HR' : res[4],
            'HL' : res[5]
        }
        out = test_planner.get_forward_kinematics_deg('AR', test_angles)
        print "AL: ", out['AL']
        
        #print "Going to...:"
        test_angles = {
            'AR' : res[0],
            'AL' : res[1],
            'KR' : res[2],
            'KL' : res[3],
            'HR' : res[4],
            'HL' : res[5]
        }
        test_controller.go_to_pose(0.5, test_angles, False)
        steps.append(test_angles)

    i = 0
    while True:
        test_controller.go_to_pose(0.5, steps[i], False)
        i = i + 1
        if (i >= len(steps)):
            i = 0
    
except:
    print "Ooops"
    traceback.print_exc()
finally:
    print "Relaxing"
    test_controller.relax()
