#!/usr/bin/python

import time
import math
from Adafruit_I2C import Adafruit_I2C
import numpy as np

# ============================================================================
# GY_521 Breakout board; MPU6050 I2C interface
# ============================================================================

class GY_521 :
  i2c = None

  # Registers/etc.
  # R/W unless specified
  _AUX_VDDIO = 0x01
  _SMPLRT_DIV = 0x19
  _CONFIG = 0x1A
  _GYRO_CONFIG = 0x1B
  _ACCEL_CONFIG = 0x1C
  _FF_THR = 0x1D
  _FF_DUR = 0x1E
  _MOT_THR = 0x1F
  _ZRMOT_THR = 0x21
  _ZRMOT_DUR = 0x22
  _FIFO_EN = 0x23
  
  _I2c_MST_CTRL = 0x24
  _I2C_SLV0_ADDR = 0x25
  _I2C_SLV0_REG = 0x26
  _I2C_SLV0_CTRL = 0x27
  _I2C_SLV1_ADDR = 0x28
  _I2C_SLV1_REG = 0x29
  _I2C_SLV1_CTRL = 0x2A
  _I2C_SLV2_ADDR = 0x2B
  _I2C_SLV2_REG = 0x2C
  _I2C_SLV2_CTRL = 0x2D
  _I2C_SLV3_ADDR = 0x2E
  _I2C_SLV3_REG = 0x2F
  _I2C_SLV3_CTRL = 0x30
  _I2C_SLV4_ADDR = 0x31
  _I2C_SLV4_REG = 0x32
  _I2C_SLV4_DO = 0x33
  _I2C_SLV4_CTRL = 0x34
  _I2C_SLV4_DI = 0x35 #R
  _I2C_MST_STATUS = 0x36 #R

  _INT_PIN_CFG = 0x37
  _INT_ENABLED = 0x38
  _INT_STATUS = 0x3A #R

  # All, naturally, read-only:
  _ACCEL_XOUT_H = 0x3B 
  _ACCEL_XOUT_L = 0x3C
  _ACCEL_YOUT_H = 0x3D
  _ACCEL_YOUT_L = 0x3E
  _ACCEL_ZOUT_H = 0x3F
  _ACCEL_ZOUT_L = 0x40
  _TEMP_OUT_H = 0x41
  _TEMP_OUT_L = 0x42
  _GYRO_XOUT_H = 0x43
  _GYRO_XOUT_L = 0x44
  _GYRO_YOUT_H = 0x45
  _GYRO_YOUT_L = 0x46
  _GYRO_ZOUT_H = 0x47
  _GYRO_ZOUT_L = 0x48

  # Read-only:
  _EXT_SENS_DATA_00 = 0x49
  _EXT_SENS_DATA_01 = 0x4A
  _EXT_SENS_DATA_02 = 0x4B
  _EXT_SENS_DATA_03 = 0x4C
  _EXT_SENS_DATA_04 = 0x4D
  _EXT_SENS_DATA_05 = 0x4E
  _EXT_SENS_DATA_06 = 0x4F
  _EXT_SENS_DATA_07 = 0x50
  _EXT_SENS_DATA_08 = 0x51
  _EXT_SENS_DATA_09 = 0x52
  _EXT_SENS_DATA_10 = 0x53
  _EXT_SENS_DATA_11 = 0x54
  _EXT_SENS_DATA_12 = 0x55
  _EXT_SENS_DATA_13 = 0x56
  _EXT_SENS_DATA_14 = 0x57
  _EXT_SENS_DATA_15 = 0x58
  _EXT_SENS_DATA_16 = 0x59
  _EXT_SENS_DATA_17 = 0x5A
  _EXT_SENS_DATA_18 = 0x5B
  _EXT_SENS_DATA_19 = 0x5C
  _EXT_SENS_DATA_20 = 0x5D
  _EXT_SENS_DATA_21 = 0x5E
  _EXT_SENS_DATA_22 = 0x5F
  _EXT_SENS_DATA_23 = 0x60
  _MOT_DETECT_STATUS = 0x61

  # R/W again
  _I2C_SLV0_DO = 0x63
  _I2C_SLV1_DO = 0x64
  _I2C_SLV2_DO = 0x65
  _I2C_SLV3_DO = 0x66
  _I2C_MST_DELAY_CTRL = 0x67
  _SIGNAL_PATH_RESET = 0x68
  _MOT_DETECT_CTRL = 0x69
  _USER_CTRL = 0x6A
  _PWR_MGMT_1 = 0x6B
  _PWR_MGMT_2 = 0x6C
  _FIFO_COUNTH = 0x72
  _FIFO_COUNTL = 0x73
  _FIFO_R_W = 0x74
  _WHO_AM_I = 0x75

  # Default address is 0x68; if AD0 high, 0x69.
  def __init__(self, address=0x68, debug=False):
    self.i2c = Adafruit_I2C(address)
    self.address = address
    self.debug = debug
    if (self.debug):
      print "Taking MPU6050 out of sleep mode"
    self.i2c.write8(self._PWR_MGMT_1, 0x00)
    time.sleep(0.001)
    
  def read_accel(self):
    ''' Returns a list [x,y,z] accelerations measured
    by IMU, in m/s/s. '''
    accel_bits = self.i2c.readList(self._ACCEL_XOUT_H, 6)
    xdat = (accel_bits[0]<<8) + accel_bits[1]
    ydat = (accel_bits[2]<<8) + accel_bits[3]
    zdat = (accel_bits[4]<<8) + accel_bits[5]
    dat = [xdat, ydat, zdat]
    for d in range(len(dat)):
        if dat[d] >= 2**15:
            dat[d] -= 2**16
        # 2<<15 corresponds to 2 gs = 19.6m/s/s
        # (we're not currently supporting switching
        # sensitivity scales)
        dat[d] = 19.6* dat[d] / 2.**15 
    return dat
    
  def read_gyro(self):
    ''' Returns a list [x,y,z] rotation rates measured
    by IMU, in deg/sec. '''
    gyro_bits = self.i2c.readList(self._GYRO_XOUT_H, 6)
    xdat = (gyro_bits[0]<<8) + gyro_bits[1]
    ydat = (gyro_bits[2]<<8) + gyro_bits[3]
    zdat = (gyro_bits[4]<<8) + gyro_bits[5]
    dat = [xdat, ydat, zdat]
    for d in range(len(dat)):
        if dat[d] >= 2**15:
            dat[d] -= 2**16
        # 2<<15 corresponds to 250 deg/sec
        dat[d] = 250. * dat[d] / 2.**15
    return dat
    
if __name__ == "__main__":
    # Largely incomplete debug code down here
    print "Making test object..."
    test_comms = GY_521(debug=True)
    print "Reading some acc"
    
    acc_zero = np.array(test_comms.read_accel())
    rot_zero = np.array(test_comms.read_gyro())
    
    pos_est = np.array([0.,0.,0.])
    vel_est = np.array([0.,0.,0.])
    rot_est = np.array([0.,0.,0.])
    rot_acc = np.array([0.,0.])
    last = time.time()
    lastlast = time.time()
    i = 0
    print "\n"
    while (True):
        acc = np.array(test_comms.read_accel())
        rot = np.array(test_comms.read_gyro())
        elapsed = time.time() - last
        last = time.time()

        # Saturate at 1g
        for j in [0,1]:
            if (abs(acc[j])>=9.8):
                acc[j]/=abs(acc[j])/9.8

        c_roll = math.asin(acc[0]/9.8)*180./math.pi
        c_pitch = -math.asin(acc[1]/9.8)*180./math.pi
        
        # Now use zero'd vals for integration
        acc -= acc_zero
        rot -= rot_zero
        vel_est += acc*elapsed
        pos_est += vel_est*elapsed
        rot_est += rot*elapsed
        rot_acc = 0.9*rot_acc + 0.1*np.array([c_roll, c_pitch])
        i += 1
        if (i > 200):
            i = 0
            elapsed = time.time()-lastlast
            lastlast=time.time()
            print "\n"*20, "Vel: ", vel_est, "\nPos: ", pos_est, \
                  "\nRot: ", rot_est, "\nARo: ", rot_acc,\
                  "\nSample Rate: ", 100./elapsed, \
                  "\n"*5
        


