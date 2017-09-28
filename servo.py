#!/usr/bin/env python3

import logging
import math
import time

#import RPi.GPIO as gpio
#import RPIO.PWM as pwm
import pigpio


class Servo:
    #PWM_FREQ= 50 # hz
    #PERIOD = PWM_FREQ / 60 # 0.02 secs
    NEUTRAL = 1500 # 1.5 ms
    MIN = 500 # 0.5 ms
    MAX = 2500 # 2.5 ms
    STEPS_DEG = (MAX - MIN) / 180
    PIN = 18

    @property
    def pulse_width(self):
        return self.p.get_servo_pulsewidth(self.PIN)

    
    @pulse_width.setter
    def pulse_width(self, pw: int):
        self.p.set_servo_pulsewidth(self.PIN, pw)

    def __init__(self, debug=False):
        if debug:
                logging.basicConfig(level=logging.DEBUG)

        logging.info('Initializing servo...')

        for attr in ['NEUTRAL', 'MIN', 'MAX', 'STEPS_DEG']:
                logging.DEBUG('{} {}'.format(attr, getattr(self, attr)))

        self.p = pigpio.pi()
        self.p.set_mode(self.PIN, pigpio.OUTPUT) 
        self.pulse_width = self.NEUTRAL
        self.p.set_servo_pulsewidth(self.PIN, self.pulse_width)

    def pw_to_deg(self, pw):
        '''
        Converts pulse width to a degree
        ''' 
        return (pw - self.MIN) / self.STEPS_DEG

    def increment(self, steps: int=1) -> float:
        '''
        Increment pulse_width by STEPS_DEG, returns angle (90 deg is neutral)
        '''
        self.pulse_width += steps * self.STEPS_DEG
        logging.debug(
                'Increased pulse width to {}'.format(self.pulse_width))
                        
        degs = self.pw_to_deg(self.pulse_width)
        logging.info('At {} deg'.format(degs))
        return degs

    def reset_pos(self) -> None:
        self.pulse_width = self.NEUTRAL
        deg = pw_to_deg(self.pulse_width)
        logging.info('Servo reset to {} deg', deg)

    def test(self):
        for i in range(10):
                time.sleep(1)
                self.increment(5)
