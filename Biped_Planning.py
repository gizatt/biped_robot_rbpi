#!/usr/bin/python

import time, random
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy import optimize

#******************************************************************************
# Biped Planner - Motion-level planning & control
#  May eventually get merged with controller, or be
#  accessible from it.
#
# We define the coodinate system of the robots
#  with +x pointing right, +y pointing forward,
#  and +z pointing forward. Thus, the hip
#  and knee are joints in x, and the ankle a joint
#  in y.
#
# Biped Kinematics wraps kinematics calculations
#  in the frame of either ankle, allowing calculation
#  of the position of the root of the robot, 
#  the other ankle, or any other part of the robot.
#******************************************************************************

class Biped_Kinematics:
    # Reasonably hard-coded for now

    # Joints we care about, in order from right
    # ankle to left. This list must indicate the
    # ROOT of the robot, on either side of which
    # is a leg.
    joints = ['AR', 'KR', 'HR', 'ROOT', 'HL', 'KL', 'AL']

    # Rough estimates of the link ascending from
    # each joint, in inches. (These should be
    # made more precise eventually...)
    link_lengths = {
        # Ankle to knee:
        'AR' : matrix("[0; 0; 3]"),
        'AL' : matrix("[0; 0; 3]"),
        # Knee to hip:
        'KR' : matrix("[0; 0; 2.4]"),
        'KL' : matrix("[0; 0; 2.4]"),
        # Hip to root:
        'HR' : matrix("[-1.5; 0; 1]"),
        'HL' : matrix("[1.5; 0; 1]")
    }

    # Rotation axes of each joint. We'll assume
    # just x/y/z and not combination for now, to 
    # keep the math simpler...
    joint_axis = {
        'AR' : 'y',
        'AL' : 'y',
        'KR' : 'x',
        'KL' : 'x',
        'HR' : 'x',
        'HL' : 'x'
    }
    
    # Whether we need to flip angle on rotation
    # axis for the given joint. (some of their
    # positive angle directions are defined in
    # silly directions... this should eventually
    # be fixed)
    joint_signs = {
        'AR' : 1.,
        'AL' : 1.,
        'KR' : -1.,
        'KL' : -1.,
        'HR' : -1.,
        'HL' : -1.
    }

    def __init__(self, debug=False):
        # No real init to do, this class mostly
        # wraps math to do on the fly...
        pass
        
    def get_rot_for_joint_ascending(self, joint, angle):
        ''' Return a 3x3 scipy matrix representing
            rotation for this joint at this angle as 
            needed for going up a leg. Angle must be
            in radians!'''
        # If it's root, we apply a flip
        if (joint == 'ROOT'):
            return matrix([
                    [-1, 0, 0],
                    [0, -1, 0],
                    [0, 0, -1]
                          ])
        sign = self.joint_signs[joint]
        angle = angle * sign
        if self.joint_axis[joint] == 'x':
            return matrix([
                    [1, 0, 0],
                    [0, cos(angle), -sin(angle)],
                    [0, sin(angle), cos(angle)]
                          ])   
        elif self.joint_axis[joint] == 'y':
            return matrix([
                    [cos(angle), 0, sin(angle)],
                    [0, 1, 0],
                    [-sin(angle), 0, cos(angle)]
                          ])
        elif joint_axis[joint] == 'z':
            # untested, robot doesn't have this yet.
            # but this might in theory work.
            return matrix([
                    [cos(angle), -sin(angle), 0],
                    [-sin(angle), cos(angle), 0],
                    [0, 0, 1]
                          ])
    def get_rot_for_joint_descending(self, joint, angle):
        ''' Equivalent to get_rot_for_joint_ascending, but
            used when going down a leg. '''
        return self.get_rot_for_joint_ascending(joint, -angle)

    def get_forward_kinematics(self, joint, angles):
        ''' Returns a dictionary of SciPy vectors
            of each joint's position, when everything
            has assumed supplied angles (in radians),
            relative to indicated joint. Joint may
            be ROOT. '''
        if (joint not in self.joints):
            print "Invalid joint."
            return
        if (len(angles) != len(self.joints)-1 and
            len(angles) != len(self.joints)):
            print "Invalid # of angles."
            return

        # Throw in root to the angles dictionary
        # to make later loops cleaner
        # The angle doesn't matter, it's a fixed
        # hard-coded coordinate flip as we switch
        # legs.
        angles['ROOT'] = 0

        # Find our position in the ordered list
        # of joints.
        start_ind = self.joints.index(joint)
        # And where the root is
        root_ind = self.joints.index('ROOT')

        # We can start off dict easy: whatever
        # joint we're given is origin.
        ret_dict = { joint: matrix([[0],[0],[0]])}

        # Try to iterate in both directions
        # through the list: left and right.

        # Left:
        # Start our rotation off as nothing
        last_rot = matrix([[1,0,0],[0,1,0],[0,0,1]])
        # If our index is less than the
        # to the root index, we'll immediately
        # be descending.
        if start_ind <= root_ind:
            state = 'desc'
        else:
            state = 'asc'
        # For every joint except final one (whose
        # rotation is irrelevant):
        for i in xrange(start_ind-1, -1, -1):
            # Take position of most recent joint
            last_pos = ret_dict[self.joints[i+1]]
            # The contortion applied for this joint
            # is, if we're ascending, the link length
            # of the last link rotated by, in reverse
            # order, the ascending rotation of every
            # joint back to the origin. Equivalently,
            # the ascending rotation for the last joint
            # times the last rot used. 
            if (state == 'asc'):
                # However, we're going through joints
                # backwards, so use opposite rotation 
                # as normal.
                last_rot = last_rot* \
                    self.get_rot_for_joint_descending(self.joints[i+1],
                        angles[self.joints[i+1]])
                ret_dict[self.joints[i]] = \
                    last_pos + last_rot*self.link_lengths[self.joints[i+1]]
            else:
            # If we're descending, same deal, but now we
            # use link length from THIS joint.
                last_rot = last_rot* \
                    self.get_rot_for_joint_ascending(self.joints[i+1],
                        angles[self.joints[i+1]])
                ret_dict[self.joints[i]] = \
                    last_pos + last_rot*self.link_lengths[self.joints[i]] 
            # If this was root, we're now descending
            if self.joints[i]=='ROOT':
                state = 'desc'

        # Right:
        # Start our rotation off as nothing
        last_rot = matrix([[1,0,0],[0,1,0],[0,0,1]])
        # If our index is greater than or equal
        # to the root index, we'll immediately
        # be descending.
        if start_ind >= root_ind:
            state = 'desc'
        else:
            state = 'asc'
        # For every joint except final one (whose
        # rotation is irrelevant):
        for i in xrange(start_ind+1, len(self.joints), 1):
            # Take position of most recent joint
            last_pos = ret_dict[self.joints[i-1]]
            # The contortion applied for this joint
            # is, if we're ascending, the link length
            # of the last link rotated by, in reverse
            # order, the ascending rotation of every
            # joint back to the origin. Equivalently,
            # the ascending rotation for the last joint
            # times the last rot used. 
            if (state == 'asc'):
                last_rot = last_rot* \
                    self.get_rot_for_joint_ascending(self.joints[i-1],
                        angles[self.joints[i-1]])
                ret_dict[self.joints[i]] = \
                    last_pos + last_rot*self.link_lengths[self.joints[i-1]]
            else:
            # If we're descending, same deal, but now we
            # use link length from THIS joint.
                last_rot = last_rot* \
                    self.get_rot_for_joint_descending(self.joints[i-1],
                        angles[self.joints[i-1]])
                ret_dict[self.joints[i]] = \
                    last_pos + last_rot*self.link_lengths[self.joints[i]] 
            # If this was root, we're now descending
            if self.joints[i]=='ROOT':
                state = 'desc'

        # That should be it!
        return ret_dict

    def get_forward_kinematics_deg(self, joint, angles):
        ''' Same as get_forward_kinematics, but converts
        angles from degrees to radians first.'''
        for key in angles.keys():
            angles[key] = angles[key]*pi/180.
        return self.get_forward_kinematics(joint, angles)

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
    return err
def ground_constraint(angs, targ_al, test_planner):
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
    outval = 0.0
    for ang in out.keys():
        if out[ang][2] <= 0:
            outval += -1*float(out[ang][2])
    return outval

# Test code if this file is being run and not imported
if __name__ == "__main__":
    test_planner = Biped_Kinematics(debug=True)
    test_angles = {
        'AR' : 0,
        'AL' : 0,
        'KR' : 0,
        'KL' : 0,
        'HR' : 50,
        'HL' : 50
    }
    test_angles_2 = {
        'AR' : 35,
        'AL' : -35,
        'KR' : -50,
        'KL' : 0,
        'HR' : 50,
        'HL' : 0
    }
    out = test_planner.get_forward_kinematics_deg('AR', test_angles)
    for joint in test_planner.joints:
        print joint, ": ", out[joint]

    sys.exit(0)

    print "Brute force testing a bunch of random angles..."
    random.seed()
    iters = 1000
    starttime = time.clock()
    for i in xrange(0, iters, 1):
        for angle in test_angles.keys():
            test_angles[angle] = random.randint(-90, 90)
        test_planner.get_forward_kinematics_deg('ROOT', test_angles)
    tottime = time.clock() - starttime
    print "Done: ", iters, " in ", tottime, " seconds"
    print "i.e. ", iters/tottime, " per sec"

    print "Trying a minimization"
    targ_al = matrix([[-2.9],[-1.8],[1.0]])
    print "Target: ", targ_hl
    res = optimize.fmin_slsqp(objective_func_sixdof, 
        [0.,0.,0.,0.,0.,0.],#[35.,-35.,-50.,0.,50.,0.], 
        [],
        args=(targ_al, test_planner),
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
