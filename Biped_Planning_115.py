#!/usr/bin/python

import time, random
from numpy import *
import scipy as Sci
import scipy.linalg
from scipy import optimize
from copy import deepcopy

#******************************************************************************
# Biped Planner - Motion-level planning & control.
#  Alternate version developed for ME115a, using
#  cleaner notation and formalisms, and now 
#  including Jacobian info! I wrap IK into here too,
#  using a pretty silly gradient descent using the
#  Jacobian to get to a pose efficiently, with some
#  bells and whistles to try to get around singularities
#  and minima...
# 
# Should eventually be merged into (i.e. replace) Biped_Planning.py.
#  (or take over for it).
#
# We define the coodinate system of the robots
#  with +x pointing right, +y pointing forward,
#  and +z pointing forward. Thus, the hip
#  and knee are joints in x, and the ankle a joint
#  in y.
#******************************************************************************

class Biped_Kinematics:
    # Reasonably hard-coded for now

    # Joints we care about, in order from left ankle
    # to right. (System is symmetric so we opt to just
    # think about everything in this direction.)
    joints = ['AL', 'KL', 'HL', 'HR', 'KR', 'AR']

    # Rough estimates of the link ascending from
    # each joint, in inches. (These should be
    # made more precise eventually...)
    ankle_to_knee = matrix("[0; 0; 3]")
    knee_to_hip = matrix("[0; 0; 2.4]")
    hip_to_hip = matrix("[3; 0; 0]")
    
    # Rotation axis of each joint.
    joint_axis = {
        'AR' : matrix("[0; 1; 0]"),
        'AL' : matrix("[0; -1; 0]"),
        'KR' : matrix("[1; 0; 0]"),
        'KL' : matrix("[-1; 0; 0]"),
        'HR' : matrix("[1; 0; 0]"),
        'HL' : matrix("[-1; 0; 0]")
    }
    
    # Rot limits for each joint
    # (these need to be synchronized across classes here,
    # they're maintained in Biped_Walker but I don't want to
    # bring all that in right now... they should go in some
    # kind of "Biped_Info" class or something?
    joint_limits = {
        'AR' : (-45., 45.),
        'AL' : (-45., 45.),
        'KR' : (-90., 90.),
        'KL' : (-90., 90.),
        'HR' : (-60., 60.),
        'HL' : (-60., 60.)
    }
    
    
    def __init__(self, debug=False):
        # No real init to do, this class mostly
        # wraps math to do on the fly...
        pass
        
    def get_trans_matrix(self, axis, angle, trans):
        ''' Given a 3x1 axis and an angle in radians, and 
            a 3x1 translation,
            returns a 4x4 transformation matrix. '''
        # Copied pretty blatantly from wikipedia
        ux = axis.item(0)
        uy = axis.item(1)
        uz = axis.item(2)
        c = cos(angle)
        s = sin(angle)
        rot = matrix([
            [c + (ux**2)*(1-c), ux*uy*(1-c)-uz*s, ux*uz*(1-c)+uy*s],
            [uy*ux*(1-c)+uz*s, c+(uy**2)*(1-c), uy*uz*(1-c)-ux*s],
            [uz*ux*(1-c)-uy*s, uz*uy*(1-c)+ux*s, c + (uz**2)*(1-c)]])
        ttrans = rot*trans
        return matrix([[rot[0,0], rot[0,1], rot[0,2], ttrans[[0]]],
                       [rot[1,0], rot[1,1], rot[1,2], ttrans[[1]]],
                       [rot[2,0], rot[2,1], rot[2,2], ttrans[[2]]],
                       [0,0,0,1]])
        

    def get_forward_kinematics(self, angles):
        ''' Returns list of list of 6 4x4 transformation matrices
            from left ankle to each joint; and jacobian matrix.
            Needs dict of all angles in robot. '''
            
        if (len(angles) != len(self.joints)):
            print "Invalid # of angles."
            return
        
        # Build a list of transform matrices going around from 
        # the left ankle
        rot_mats = [ self.get_trans_matrix(self.joint_axis['AL'], angles['AL'], self.ankle_to_knee),
                     self.get_trans_matrix(self.joint_axis['KL'], angles['KL'], self.knee_to_hip),
                     self.get_trans_matrix(self.joint_axis['HL'], angles['HL'], self.hip_to_hip),
                     self.get_trans_matrix(self.joint_axis['HR'], angles['HR'], -self.knee_to_hip),
                     self.get_trans_matrix(self.joint_axis['KR'], angles['KR'], -self.ankle_to_knee),
                     self.get_trans_matrix(self.joint_axis['AR'], angles['AR'], matrix("[0;0;0]")) ]
        
        # Combine to get whole transform (and record at each stage)
        trans = list()
        transform = matrix("1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1")
        for t in rot_mats:
            trans.append(transform)
            transform = transform * t
        
        # And do jacobian
        # ith column is cross prod of axis of rev. with
        # vec from joint to end effector
        jac = matrix(zeros((3, 6)))
        end_pos = trans[-1]*matrix("0;0;0;1")
        tmp_4mat = matrix("0;0;0;1")
        for i in range(0, len(self.joints)):
            axis_pos = trans[i]*matrix("0;0;0;1")
            tmp_4mat[0:3] = self.joint_axis[self.joints[i]]
            axis_dir = trans[i]*tmp_4mat
            jac_col = cross(transpose(axis_dir[0:3]), transpose(end_pos[0:3]-axis_pos[0:3]))
            jac[0:3, i] = transpose(jac_col)
            
        return (trans, jac)

    def get_forward_kinematics_deg(self, angles):
        ''' Same as get_forward_kinematics, but converts
        angles from degrees to radians first.'''
        new_angles = deepcopy(angles)
        for key in angles.keys():
            new_angles[key] = angles[key]*pi/180.
        return self.get_forward_kinematics(new_angles)


class Biped_Planning_6DOF:
    # Wraps some useful planning functions relying
    # on, and optimizing on, the kinematics model.

    kin_model = None
    
    def __init__(self, debug=False):
        # Not much here, this just wraps convenient functions.
        # Instantiating kin model is most of it
        self.kin_model = Biped_Kinematics(debug)
        random.seed()
        
    def inverse_kin_left_leg(self, target_pos, maxattempts=1, alpha=1.0, alphadecay = 1.0, reg = 0.0, eps=0.01, delta_eps=0.001):
        ''' Given a target pos (in frame of left ankle),
            returns dict of joint angles that hopefully
            get close to the desired position. Target_pos
            should be given as a column numpy matrix (3x1) '''
        
        # Generate a guess
        print maxattempts
        for attempt in range(1, maxattempts+1):
            attempts = 1
            guess_angles = {
                'AR' : random.uniform(-45., 45.),
                'AL' : random.uniform(-45., 45.),
                'KR' : random.uniform(-45., 45.),
                'KL' : random.uniform(-45., 45.),
                'HR' : random.uniform(-45., 45.),
                'HL' : random.uniform(-45., 45.)
            }
            # Try it
            res = self.kin_model.get_forward_kinematics_deg(guess_angles)
            trans = res[0]
            jac = res[1]
            res_pos = trans[-1]*matrix("0;0;0;1")
            err = res_pos[0:3] - target_pos[0:3]
            olderr = err
            delta_err = matrix("100000.0; 1000000.0; 1000000.0")
            pocket = guess_angles
            pocket_err = linalg.norm(err)
            while (linalg.norm(err) > eps and linalg.norm(delta_err) > delta_eps):
                # update guess
                update_guess = alpha*transpose((transpose(jac)*err))
                for i in range(0, len(self.kin_model.joints)):
                    j = self.kin_model.joints[i]
                    guess_angles[j] = guess_angles[j] - update_guess.item(i) - reg*guess_angles[j]
                    # constrain within joint limits
                    guess_angles[j] = sorted((self.kin_model.joint_limits[j][0], 
                        guess_angles[j], self.kin_model.joint_limits[j][1]))[1]
                    
                # and do it again!
                res = self.kin_model.get_forward_kinematics_deg(guess_angles)
                trans = res[0]
                jac = res[1]
                res_pos = trans[-1]*matrix("0;0;0;1")
                err = res_pos[0:3] - target_pos[0:3]
                delta_err = err - olderr
                olderr = err
                print "Err: ", linalg.norm(err)
                alpha = alpha * alphadecay
                if (linalg.norm(err) < pocket_err):
                    pocket_err = linalg.norm(err)
                    pocket = deepcopy(guess_angles)
                attempts = attempts + 1
        
            if (attempt == 1):
                best_guess = deepcopy(pocket)
                best_err = pocket_err
            else:
                if (pocket_err < best_err):
                    best_guess = deepcopy(pocket)
                    best_err = pocket_err
            print "Best yet: "
            print best_guess
            print "Err: ", best_err
            print "Attempts: ", attempts
                
        print "\n\n\n Returning:"
        print best_guess
        print "Err: ", best_err
        return best_guess
            
# Test code if this file is being run and not imported
if __name__ == "__main__":
    #test_kin = Biped_Kinematics(debug=True)
    # 
    #joint_angles = {
    #    'AR' : 0,
    #    'AL' : 45,
    #    'KR' : 0,
    #    'KL' : 0,
    #    'HR' : 0,
    #    'HL' : 0
    #}
    #res = test_kin.get_forward_kinematics_deg(joint_angles)
    #print "Trans: ", res[0][-1]
    #print "\n\nJac: ", res[1]
    test_planner = Biped_Planning_6DOF()
    #print test_planner.figure_out_pose('AR', 'AL', [-2.0, 0.0, 1.0], debug=True)
    print test_planner.inverse_kin_left_leg(matrix("3;0;1"), alpha=0.8)
