#!/usr/bin/python

from Adafruit_PWM_Servo_Driver import PWM
import time

#******************************************************************************
# Biped Walker -- Motor Interface Code
#  Abstracts servo command away so that motion planning code
#  doesn't need to worry about it.
#  Defines servos by two-char names, being body part and then
#  side of body. Body parts are Ankle (A), Knee (K), and Hip (H).
#  Side of body is Right or Left.
#  Angles per joint are defined:
#    Ankle: 0 degree is flat on ground. -90 is bottom of foot
#    pointing outwards. 90 is bottom of foot pointing inwards.
#    (Due to frame of foot, angle of 90 or -90 can't actually be
#    achieved without self-collision.)
#    Knee: 0 degree is a straight knee. -90 is foot behind knee.
#    +90 is foot in front of knee.
#    Hip: To maximize range of motion in both directions, we'll
#    define 0 degree to be the servo being at a 45 degree angle
#    with the control backpack / board facing up and backwards.
#    +90 degrees is the bottom of the foot facing straight forward
#    (assuming straight knee), -90 degree is the bottom of the foot
#    facing straight backward. Again, due to self-collision
#    these angles can't be achieved.
#******************************************************************************

class Biped_Robot :
    # PWM device
    pwm = None

    # Name of each joint
    servos = ['AR', 'KR', 'HR', 'AL', 'KL', 'HL']

    # Servo pins
    # Diction: key is two-char servo name,
    # value is pin servo is on.
    servo_pins = {}
    # (entries added in __init__)
    
    # Servo calibration ranges
    # Dictionary: key is two-char servo name,
    # value is pulse length out of 4096 to get
    # servo to -45 and 45 deg respectively
    servo_calib = {}
    # (entries added in __init__)

    # Translation values from angle to
    # pulse-len. Derived from calib values,
    # set up in __init__. Value pair is
    # (zeropoint, pulselen per degree), from
    # which we can interpolate out to get any
    # angle command.
    servo_translate = {}

    # Servo range limits that we'll impose.
    # These are just shy of mechanical limits;
    # they keep power from being wasted trying to
    # achieve impossible contortions. Each
    # tuple encodes lower, then upper, angle limits
    # on each joint.
    servo_limits = {}
    # (entries added in __init__)
    
    def __init__(self, address=0x40, debug=False):
        self.pwm = PWM(0x40, debug=debug);
        # Set frequency to 60 Hz
        self.pwm.setPWMFreq(60)

        #Populate servo calibration from hand-tuned
        # values, as well as pins
        self.servo_pins['AR'] = 0
        self.servo_calib['AR'] = (300, 525)
        self.servo_limits['AR'] = (-45, 45)
        self.servo_pins['AL'] = 4
        self.servo_calib['AL'] = (500, 265)
        self.servo_limits['AL'] = (-45, 45)
        
        self.servo_pins['KR'] = 1
        self.servo_calib['KR'] = (250, 495)
        self.servo_limits['KR'] = (-90, 90)
        self.servo_pins['KL'] = 5
        self.servo_calib['KL'] = (510, 280)
        self.servo_limits['KL'] = (-90, 90)

        self.servo_pins['HR'] = 2
        self.servo_calib['HR'] = (645, 380)
        self.servo_limits['HR'] = (-45, 60)
        self.servo_pins['HL'] = 6
        self.servo_calib['HL'] = (150, 385)
        self.servo_limits['HL'] = (-45, 60)

        # Calculate servo translation values.
        for servo in self.servos:
            servo_zeropoint = \
                (self.servo_calib[servo][0] + self.servo_calib[servo][1])/2.
            servo_pulselenperdeg = \
                (self.servo_calib[servo][1] - self.servo_calib[servo][0])/90.
            self.servo_translate[servo] = \
                (servo_zeropoint, servo_pulselenperdeg)
        
        # For now, relax all servos.
        for servo in self.servos:
            self.pwm.setPWM(self.servo_pins[servo], 0, 0)
        
    def get_servos(self):
        ''' Return a list of all servo names. '''
        return self.servos

    def relax(self):
        ''' Relax all servos. '''
        for servo in self.servos:
            self.pwm.setPWM(self.servo_pins[servo], 0, 0)

    def zero(self):
        ''' Set all servos to zero angle.'''
        for servo in self.servos:
            self.set_angle(servo, 0)
            
    def set_angle(self, servo, angle, debug=False):
        ''' Set angle of a given servo. '''
        if (servo not in self.servo_translate):
            print "Invalid servo."
            return;        
        if (angle > self.servo_limits[servo][1]+1 \
            or angle < self.servo_limits[servo][0]-1):
            print "Angle %d out of range for this servo %s." % (angle, servo)
            return;
        
        # Interpolate from calibration points to achieve
        # angle.
        angle_command = int(self.servo_translate[servo][0] + \
            self.servo_translate[servo][1]*angle)
        if (debug):
            print "Commanding servo %s: angle %d via command %d\n" % \
                (servo, angle, angle_command)
        self.pwm.setPWM(self.servo_pins[servo], 0, angle_command)

    def set_angles(self, angles, debug=False):
        ''' Set angle of all servos, by supplying list of angles
            corresponding to each angle in servos list.'''
        if (angles.count() != self.servos.count()):
            print "Incorrect # of angles for servos."
            return
        
        for i in range(0, angles.count()):
            self.set_angle(self.servos[i], angles[i], debug=debug)

# Test code if this file is being run and not imported
if __name__ == "__main__":
    test_biped = Biped_Robot(debug=True)
    print "Relaxing"
    test_biped.relax()
    print "Proceed with testing. Test unit is named test_biped."
    
    
