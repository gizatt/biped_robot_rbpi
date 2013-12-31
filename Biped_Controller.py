#!/usr/bin/python

from Biped_Walker import Biped_Robot
import time
import traceback

#******************************************************************************
# Biped Controller -- Control code.
#   Provides useful functionality for controlling the entire
#   robot.
#******************************************************************************

class Biped_Controller :
    # Biped walker
    walker = None
    # Current pose, accurate to long time scale (Does not
    #  update until end of a gradual pose change)
    curr_pose = {}
    
    def __init__(self, address=0x40, debug=False):
        # Make our walker
        self.walker = Biped_Robot(address, debug)
        # Start in zero'd pose
        self.walker.zero()
        for servo in self.walker.servos:
            self.curr_pose[servo] = 0
            
    def hold_pose(self, dur, debug=False):
        ''' Holds current pose for specified duration. '''
        self.go_to_pose(dur, self.curr_pose, debug)
        
    def go_to_pose(self, dur, angles, debug=False):
        ''' Goes to the specified pose from the current one
            over the specified number of seconds.'''
        for servo in self.walker.servos:
            if servo not in angles:
                print "Invalid angles, not all servos present."
                return
            
        starttime = time.time()
        if (debug):
            i = 0
        while (time.time() < starttime + dur):
            frac = (time.time()-starttime)/dur
            if (debug):
                i = i + 1
                print "Iter %d, frac %f" % (i, frac)
            for servo in self.walker.servos:
                interp_ang = frac*float(angles[servo]) + \
                    (1.-frac)*float(self.curr_pose[servo])
                self.walker.set_angle(servo, interp_ang, debug)
                
        if (debug):
            print "Done."
        for servo in self.walker.servos:
            self.curr_pose[servo] = angles[servo]

    def relax(self):
        ''' Relaxes robot.'''
        self.walker.relax()

# Test code if this file is being run and not imported
if __name__ == "__main__":
    print "Spawning biped controller"
    test_controller = Biped_Controller(debug=True)

    TIMESTEP = 0.25
    
    try:
        print "Crouching"
        next_step = { 'AR': 0, 'KR': -30, 'HR': 30,
                      'AL': 0, 'KL': -30, 'HL': 30 }
        test_controller.go_to_pose(TIMESTEP, next_step, False)

        print "Shifting feet to starting pose"
        next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                      'AL': 0, 'KL': 30, 'HL': 0 }
        test_controller.go_to_pose(TIMESTEP, next_step, False)

        while (1):
            print "Standing to right foot"
            next_step = { 'AR': 35, 'KR': -50, 'HR': 50,
                          'AL': -35, 'KL': 0, 'HL': 0 }
            test_controller.go_to_pose(TIMESTEP, next_step, False)
            
            print "Advancing left foot"
            next_step = { 'AR': 30, 'KR': 30, 'HR': 0,
                          'AL': 0, 'KL': -30, 'HL': 60 }
            test_controller.go_to_pose(TIMESTEP, next_step, False)

            print "Dropping left foot"
            next_step = { 'AR': 0, 'KR': 40, 'HR': -10,
                          'AL': 0, 'KL': -30, 'HL': 60 }
            test_controller.go_to_pose(TIMESTEP, next_step, False)

            print "Standing to left foot"
            next_step = { 'AR': -35, 'KR': 0, 'HR': 0,
                          'AL': 35, 'KL': -50, 'HL': 50 }
            test_controller.go_to_pose(TIMESTEP, next_step, False)
            
            print "Advancing right foot"
            next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                          'AL': 30, 'KL': 40, 'HL': -10 }
            test_controller.go_to_pose(TIMESTEP, next_step, False)

            print "Dropping right foot"
            next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                          'AL': 0, 'KL': 30, 'HL': 0 }
            test_controller.go_to_pose(TIMESTEP, next_step, False)
        
        
        test_controller.hold_pose(10)
        
    except:
        print "Ooops"
        traceback.print_exc()
    finally:
        print "Relaxing"
        test_controller.relax()
