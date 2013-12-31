#!/usr/bin/python

from Biped_Controller import Biped_Controller
from Biped_Planning import all
import time
import traceback
import sys

# Simple script to just go to a particular point
# as derived by IK
if len(sys.argv) != 4:
    print "Usage: "
    print "\t--X to place left foot at"
    print "\t--Y to place left foot at"
    print "\t--Z to place left foot at"
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
    targ_hl = matrix([[x],[y],[z]])
    print "Target: ", targ_hl
    res = optimize.fmin_slsqp(objective_func_sixdof, 
        [0.,0.,0.,0.,0.,0.],#[35.,-35.,-50.,0.,50.,0.], 
        [],
        args=(targ_hl, test_planner),
        bounds=[(-60,60),(-60,60),(-60,60),(-60,60),(-60,60),(-60,60)],
        iter=1000000,
        iprint=1,
        epsilon=1.0)
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

    print "Going..."
    next_step = { 'AR': 0, 'KR': -30, 'HR': 30,
                  'AL': 0, 'KL': -30, 'HL': 30 }
    test_controller.go_to_pose(5, next_step, False)

except:
    print "Ooops"
    traceback.print_exc()
finally:
    print "Relaxing"
    test_controller.relax()