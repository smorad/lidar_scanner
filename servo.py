#!/usr/bin/env python3

# Library for controlling the Z (or rho) axis servo of the lidar scanner

import logging
import math
import time
import subprocess
from contextlib import contextmanager
import pigpio


class Pigpiod:
    '''
    Pigpiod is used to get hardware PWM on the raspberry pi. The
    pi provided PWM sucks. It is not precise which causes small jitters
    in the servo, which causes scan artifacts.
    '''

    def __enter__(self):
        self.pigpio_server = subprocess.Popen('/usr/local/bin/pigpiod')
        return self.pigpio_server

    def __exit__(self, *args):
        self.pigpio_server.terminate()


class Servo:
    '''
    Servo controller class
    '''
    NEUTRAL = 1500 # 1.5 ms
    MIN = 500 # 0.5 ms
    MAX = 2500 # 2.5 ms
    STEPS_DEG = (MAX - MIN) / 180
    PIN = 18

    @property
    def pulse_width(self) -> float:
        return self.p.get_servo_pulsewidth(self.PIN)

    
    @pulse_width.setter
    def pulse_width(self, pw: int) -> None:
        self.p.set_servo_pulsewidth(self.PIN, pw)

    def __init__(self, debug=False):
        '''
        Initialize servo hardware
        '''
        if debug:
                logging.basicConfig(level=logging.DEBUG)

        logging.info('Initializing servo...')

        for attr in ['NEUTRAL', 'MIN', 'MAX', 'STEPS_DEG']:
                try:
                    logging.DEBUG('{} {}'.format(attr, getattr(self, attr)))
                except:
                    pass

        self.p = pigpio.pi()
        self.p.set_mode(self.PIN, pigpio.OUTPUT) 
        self.pulse_width = self.NEUTRAL
        self.p.set_servo_pulsewidth(self.PIN, self.pulse_width)

    def pw_to_deg(self, pw) -> float:
        '''
        Converts pulse width to a degree

        By default, 180 deg is parallel to the ground and 90 deg is straight up.
        Subtract 90 so our spherical to cartesian conversion is easier later.
        ''' 
        return (pw - self.MIN) / self.STEPS_DEG - 90

    def increment(self, steps: int=1) -> float:
        '''
        Increment pulse_width by STEPS_DEG, returns angle (90 deg is neutral)
        '''
        self.pulse_width += steps * self.STEPS_DEG
        logging.debug(
                'Increased pulse width to {}'.format(self.pulse_width))
                        
        degs = self.pw_to_deg(self.pulse_width)
        logging.info('At {:.2f} deg from +Z axis'.format(degs))
        return degs

    @property
    def phase_angle(self) -> float:
        return self.pw_to_deg(self.pulse_width)

    def reset_pos(self) -> None:
        '''Reset servo position to default'''
        self.pulse_width = self.NEUTRAL
        deg = self.pw_to_deg(self.pulse_width)
        logging.info('Servo reset to {:.2f} deg from +Z axis'.format(deg))

    def test(self):
        for i in range(10):
                time.sleep(1)
                self.increment(5)
