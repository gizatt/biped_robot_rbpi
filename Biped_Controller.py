#!/usr/bin/python

from Biped_Walker import Biped_Robot
from GY_521_MPU6050 import GY_521
import thread, threading
import time
import traceback
import numpy as np
import math

#******************************************************************************
# Biped Controller -- Control code.
#   Provides useful functionality for controlling the entire
#   robot.
#******************************************************************************

class Biped_Controller :
    # Biped walker
    walker = None
    # IMU
    imu = None
    # Roll/pitch/yaw estimates
    rpy = np.array([0.,0.,0.])
    rpy_0 = np.array([0.,0.,0.])

    # Last acc and gyro measurements, for averaging
    l_a_rpy = np.array([0.0,0.0,0.0])
    l_g_rpy = np.array([0.0,0.0,0.0])
    
    # Current pose, accurate to long time scale (Does not
    #  update until end of a gradual pose change)
    curr_pose_long = {}
    # Current pose, absolute.
    curr_pose_short = {}
    # Current pose goal
    curr_pose_goal = None
    # Lock to make sure it isn't modified in middle of
    # pose pursuit
    curr_pose_goal_lock = threading.Semaphore(1)
    # Time we started getting to goal
    curr_pose_goal_tstart = None
    # Time we'll arrive at goal
    curr_pose_goal_tend = None

    # Current balance goal
    curr_balance_goal = None
    balance_horiz_joints = None
    balance_horiz_factors = None
    balance_horiz_ierr = None
    balance_forward_joints = None
    balance_forward_factors = None
    balance_forward_ierr = None
    # P and I constants
    balance_i = 0.001
    balance_p = 0.015
    # balance gradient: "desired" angles while balancing
    balance_gradient = None
    balance_grad_k = 0.03
    # Threads
    imu_thread = None
    pose_thread = None
    keep_running = True
    
    def __init__(self, address=0x40, \
                 update_pose_rate=0.01, update_imu_rate=0.001, \
                 debug=False, debug_update_pose=False, \
                 debug_update_imu=False):
        # Make our walker
        self.walker = Biped_Robot(address, debug=debug)

        # And imu
        self.imu = GY_521(debug=debug)
        
        # Start in zero'd pose
        self.walker.zero()
        
        for servo in self.walker.servos:
            self.curr_pose_long[servo] = 0
            self.curr_pose_short[servo] = 0
        
        if (debug):
            print "Waiting for zero pose to be reached..."
        time.sleep(1)

        if (debug):
            print "Calibrating roll/pitch from imu."
        t_init = time.time()
        i = 0
        rp_0 = np.array([0,0])
        while (time.time() - t_init < 2.):
            rp_0 += self.read_rp_from_imu_acc()
            i += 1.
        rp_0 /= i
        self.rpy_0[0] = rp_0[0]
        self.rpy_0[1] = rp_0[1]
        if (debug):
            print "Calibration: ", self.rpy_0

        # Spawn threads to update pose and imu
        pose_thread = thread.start_new_thread(self.update_pose,
            (update_pose_rate, debug_update_pose))
        imu_thread = thread.start_new_thread(self.update_imu,
            (update_imu_rate, debug_update_imu))        
        
    def update_pose(self, update_rate, debug=False):
        ''' Parallel thread for pose following. '''
        try:
            while self.keep_running:
                # If we're on our way to a pose goal...
                self.curr_pose_goal_lock.acquire()
                if (self.curr_pose_goal != None):
                    # Adopt appropriate intermediate pose
                    # based on current time
                    dur = self.curr_pose_goal_tend-self.curr_pose_goal_tstart
                    frac = (time.time()-self.curr_pose_goal_tstart)/dur
                    
                    # Are we done?
                    if (frac < 1.):
                        for servo in self.walker.servos:
                            interp_ang = frac*float(self.curr_pose_goal[servo]) + \
                                (1.-frac)*float(self.curr_pose_long[servo])
                            self.walker.set_angle(servo, interp_ang, debug)
                            self.curr_pose_short[servo] = interp_ang
                    else:
                        #Done!
                        self.curr_pose_goal = None
                        for servo in self.walker.servos:
                            self.curr_pose_long[servo] = self.curr_pose_short[servo]
                        self.curr_pose_goal_tstart = None
                        self.curr_pose_goal_tend = None
                    
                # Or currently balancing
                elif (self.curr_balance_goal != None):

                    # Push along gradient toward desired goal
                    if (self.balance_gradient != None):
                        for servo in self.balance_gradient:
                            curr = self.curr_pose_short[servo]
                            err = self.balance_gradient[servo]-curr
                            new = curr+err*self.balance_grad_k
                            self.curr_pose_short[servo] = new
                            self.walker.set_angle(servo, new, debug)
                        
                    # Feedback control in both dimensions using
                    # stored joints

                    # Horizontal:
                    horiz_err = self.curr_balance_goal[0]- self.rpy[0]
                    self.balance_horiz_ierr = self.balance_horiz_ierr*0.5 + horiz_err
                    for i in range(len(self.balance_horiz_joints)):
                        self.curr_pose_short[self.balance_horiz_joints[i]] += \
                            self.balance_horiz_factors[i]*(
                            self.balance_p*horiz_err + \
                            self.balance_i*self.balance_horiz_ierr)
                        self.walker.set_angle(self.balance_horiz_joints[i],
                            self.curr_pose_short[self.balance_horiz_joints[i]], debug)

                    # Forward/back:
                    forward_err = self.curr_balance_goal[1] - self.rpy[1]
                    self.balance_forward_ierr = self.balance_forward_ierr*0.5 + forward_err
                    for i in range(len(self.balance_horiz_joints)):
                        self.curr_pose_short[self.balance_forward_joints[i]] += \
                            self.balance_forward_factors[i]*(
                            self.balance_p*forward_err + \
                            self.balance_i*self.balance_forward_ierr)
                        self.walker.set_angle(self.balance_forward_joints[i],
                            self.curr_pose_short[self.balance_forward_joints[i]], debug)

                    # Copy over to perm storage:
                    for servo in self.walker.servos:
                            self.curr_pose_long[servo] = self.curr_pose_short[servo]
                            
                self.curr_pose_goal_lock.release()
                time.sleep(update_rate)
        except:
            print "Error in pose thread."
            traceback.print_exc()
        finally:
            self.relax()
        
    def update_imu(self, update_rate, debug=False):
        ''' Parallel thread for imu tracking. '''
        try:
            # If we have an IMU...
            time_last = time.time()
            i=0
            sumg = np.array([0.0,0.0,0.0])
            while (self.keep_running):
                if (self.imu != None):
                    rp = self.read_rp_from_imu_acc(debug=debug)
                    rpy = np.array([rp[0],rp[1],0]) - self.rpy_0
                    # And read similar dat from gyro
                    gyr = np.array(self.imu.read_gyro())
                    # (flip x and y axes on gyro)
                    gyr = np.array([gyr[1], gyr[0], gyr[2]])
                    
                    elapsed = time.time()-time_last
                    time_last = time.time()

                    # Average over time to remove noise
                    self.l_a_rpy = (0.5*self.l_a_rpy + \
                        0.5*rpy)
                    self.l_g_rpy = (0.1*self.l_g_rpy + \
                        0.9*gyr)

                    # And filter together to get rpy est
                    # Use gyro to predict where we should be
                    pred_rpy = self.rpy + elapsed*self.l_g_rpy
                    sumg += elapsed*self.l_g_rpy
                    
                    # Weigh that against absolute reference from acc
                    self.rpy = 0.5*(self.l_a_rpy) + \
                        0.5*pred_rpy

                    time.sleep(update_rate)
        except:
            print "Error in IMU thread:"
            traceback.print_exc()
        finally:
            self.relax()
            
    def read_rp_from_imu_acc(self, debug=False):
        # Read in acc from imu
        acc = np.array(self.imu.read_accel())
                
        # Saturate at 1g
        for j in [0,1]:
            if (abs(acc[j])>=9.8):
                acc[j]/=abs(acc[j])/9.8

        c_roll = math.asin(acc[0]/9.8)*180./math.pi
        c_pitch = -math.asin(acc[1]/9.8)*180./math.pi
        return np.array([c_roll, c_pitch])
    
    def hold_pose(self, dur, debug=False):
        ''' Holds current pose for specified duration. '''
        # Just go to the current pose for that duration
        self.go_to_pose(dur, self.curr_pose_short,)
        
    def go_to_pose(self, dur, angles, debug=False):
        ''' Goes to the specified pose from the current one
            over the specified number of seconds.'''
        for servo in self.walker.servos:
            if servo not in angles:
                print "Invalid angles, not all servos present."
                return

        # Set this to none first to make sure pose won't
        # change in separate thread after we save the
        # intermediate pose
        self.curr_pose_goal_lock.acquire()
        self.curr_balance_goal = None
        self.curr_pose_goal = None
        for servo in self.walker.servos:
            self.curr_pose_long[servo] = self.curr_pose_short[servo]

        self.curr_pose_goal_tstart = time.time()
        self.curr_pose_goal_tend = self.curr_pose_goal_tstart+dur
        self.curr_pose_goal = angles
        self.curr_pose_goal_lock.release()

    def relax(self):
        ''' Clears pose goals and relaxes robot.'''
        self.curr_pose_goal_lock.acquire()
        self.curr_pose_goal = None
        for servo in self.walker.servos:
            self.curr_pose_long[servo] = self.curr_pose_short[servo]
        self.curr_pose_goal_tstart = None
        self.curr_pose_goal_tend = None
        self.walker.relax()
        self.curr_pose_goal_lock.release()

    def balance_on_leg(self, horiz_joints, horiz_joint_factors,
            horiz_target, forward_joints, forward_joint_factors,
            forward_target, balance_gradient=None):
        ''' Closed-loop balance utilizing horiz_joints
            for left/right balance, forward_joints for
            forward/back balance. Sign dictates whether
            increasing the angle of either joint increases
            or decreases angle toward the target. '''
        self.curr_pose_goal_lock.acquire()
        self.curr_pose_goal = None
        for servo in self.walker.servos:
            self.curr_pose_long[servo] = self.curr_pose_short[servo]
        self.curr_balance_goal = [horiz_target, forward_target]
        self.balance_horiz_joints = horiz_joints
        self.balance_horiz_factors = horiz_joint_factors
        self.balance_forward_joints = forward_joints
        self.balance_forward_factors = forward_joint_factors
        self.balance_gradient = balance_gradient

        # No integral error yet
        self.balance_forward_ierr = 0.
        self.balance_horiz_ierr = 0.
        
        self.curr_pose_goal_lock.release()

    def push_toward_goal(self, goal_pose, factor, debug=False):
        ''' For a set of servos, push the system n% of the way
        to that goal. If we're pursuing a pose goal, cancels. '''
        self.curr_pose_goal_lock.acquire()
        if (self.curr_pose_goal != None):
            self.curr_pose_goal = None
        for servo in goal_pose.keys():
            curr = self.curr_pose_short[servo]
            new = goal_pose[servo]*factor + curr*(1-factor)
            self.curr_pose_short[servo] = new
            self.curr_pose_long[servo] = new
            self.walker.set_angle(servo, new, debug)
        self.curr_pose_goal_lock.release()
        
    def get_pose(self):
        new_pose = {}
        for servo in self.walker.servos:
            new_pose[servo] = self.curr_pose_short[servo]
        return new_pose

    def __del__(self):
        ''' Clean up threads '''
        self.keep_running = False
        time.sleep(0.5)
        self.relax()
        
    def simple_walk_cycle_biped(self, TIMESTEP):
        ''' Performs open-loop walking cycle. Assumes
            3-joint legs (HR/L, AR/L, KR/L) '''
        print "Crouching"
        next_step = { 'AR': 0, 'KR': -30, 'HR': 30,
                      'AL': 0, 'KL': -30, 'HL': 30 }
        self.go_to_pose(TIMESTEP, next_step, False)
        time.sleep(TIMESTEP)

        print "Shifting feet to starting pose"
        next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                      'AL': 0, 'KL': 30, 'HL': 0 }
        self.go_to_pose(TIMESTEP, next_step, False)
        time.sleep(TIMESTEP)
        
        while (1):
            print "Standing to right foot"
            next_step = { 'AR': 35, 'KR': -50, 'HR': 50,
                          'AL': -35, 'KL': 0, 'HL': 0 }
            self.go_to_pose(TIMESTEP, next_step, False)
            time.sleep(TIMESTEP)
            
            print "Advancing left foot"
            next_step = { 'AR': 30, 'KR': 30, 'HR': 0,
                          'AL': 0, 'KL': -30, 'HL': 60 }
            self.go_to_pose(TIMESTEP, next_step, False)
            time.sleep(TIMESTEP)
            
            print "Dropping left foot"
            next_step = { 'AR': 0, 'KR': 40, 'HR': -10,
                          'AL': 0, 'KL': -30, 'HL': 60 }
            self.go_to_pose(TIMESTEP, next_step, False)
            time.sleep(TIMESTEP)
            
            print "Standing to left foot"
            next_step = { 'AR': -35, 'KR': 0, 'HR': 0,
                          'AL': 35, 'KL': -50, 'HL': 50 }
            self.go_to_pose(TIMESTEP, next_step, False)
            time.sleep(TIMESTEP)
            
            print "Advancing right foot"
            next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                          'AL': 30, 'KL': 40, 'HL': -10 }
            self.go_to_pose(TIMESTEP, next_step, False)
            time.sleep(TIMESTEP)
            
            print "Dropping right foot"
            next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                          'AL': 0, 'KL': 30, 'HL': 0 }
            self.go_to_pose(TIMESTEP, next_step, False)
            time.sleep(TIMESTEP)
        
        self.hold_pose(10)
        time.sleep(10)


            
# Test code if this file is being run and not imported
if __name__ == "__main__":
    print "Spawning biped controller"
    test_controller = Biped_Controller(debug=True)

    TIMESTEP = 2
    TIMESTEP_BAL = 0.5
    pose = { 'AR': 35, 'KR': -50, 'HR': 50,
             'AL': -35, 'KL': -30, 'HL': 30 }
    
    try:
        print "Crouching"
        next_step = { 'AR': 0, 'KR': -30, 'HR': 30,
                      'AL': 0, 'KL': -30, 'HL': 30 }
        test_controller.go_to_pose(TIMESTEP, next_step, False)
        time.sleep(TIMESTEP)

        print "Shifting feet to starting pose"
        next_step = { 'AR': 0, 'KR': -30, 'HR': 60,
                      'AL': 0, 'KL': 30, 'HL': 0 }
        test_controller.go_to_pose(TIMESTEP, next_step, False)
        time.sleep(TIMESTEP)
        
        print "Standing to right foot"
        next_step = { 'AR': 35, 'KR': -50, 'HR': 50,
                      'AL': -35, 'KL': 0, 'HL': 0 }
        test_controller.go_to_pose(TIMESTEP, next_step, False)
        time.sleep(TIMESTEP)
        
        print "Raising left foot"
        next_step = { 'AR': 35, 'KR': -50, 'HR': 50,
                      'AL': -35, 'KL': -50, 'HL': 50 }
        test_controller.go_to_pose(TIMESTEP, next_step, False)
        time.sleep(TIMESTEP)
        
        test_controller.balance_on_leg(['AR'], [1], 30, \
            ['KR', 'HR'], [-1, -1], 0)

        # This would perform balance on left foot
        #test_controller.balance_on_leg(['AL'], [-1.], -30, \
        #    ['KL', 'HL'], [-1, -1], 0)
        
        while (True):
            #spinspinspin
            pass
        
        # This would do the walk cycle
        #test_controller.simple_walk_cycle(TIMESTEP)
        
    except:
        print "Ooops"
        traceback.print_exc()
    finally:
        print "Relaxing"
        test_controller.relax()
        test_controller.__del__()
        time.sleep(1)
