#!/usr/bin/env python3

import time
import math
import pickle
import os
import logging

import RPi.GPIO as gpio


duty_cycle = length / period


class Servo:
    PWM_FREQ= 50 # hz
    PERIOD = self.PWM_FREQ / 60 # 0.02 secs
    NEUTRAL = 1.5 # ms
    DUTY_CYCLE =  self.MIN / self.PERIOD
    MIN = 500 * 10 ** -3 # in ms
    MAX = 2500 * 10 ** -3 # in ms
    STEP = 3 * 10 ** -3 # in ms
    STEPS_DEG = (self.MAX - self.MIN) / 180

    def __init__(self):
        logging.info('Initializing servo...')
        gpio.setmode(gpio.BOARD)
        gpio.setup(12, gpio.OUT)

        self.p = gpio.PWM(12, pwm_freq)
        self.pos = self.NEUTRAL
        self.p.start(self.pos)

    def increment_deg(deg: float=1) -> float:
        '''
        Increment in deg, return current angle
        '''
        self.pos += deg
        p.ChangeDutyCycle(self.pos_to_dc())
        logging.info('Servo set to %f deg', self.pos)
        assert(math.abs(duty_cycle - p.GetDutyCycle < 0.001))
        return p.GetDutyCycle()

    def reset_pos(self): -> None:
        self.pos = self.NEUTRAL
        self.p.ChangeDutyCycle(self.pos_to_dc(self.pos))
        logging.info('Servo reset to %f deg', self.pos)

    def pos_to_dc(self) -> float:
        '''
        Returns the current servo position in terms of
        duty cycle (pct)
        '''
        return self.ms_to_pct(self.deg_to_ms(self.pos))

    def deg_to_ms(self, deg: float) -> float:
        '''
        Convert a degree number to a PWM phase length
        '''
        return deg * self.STEPS_DEG 

    def ms_to_pct(self, ms: float) -> float:
        '''
        Convert a time in millis to a PWM percentage
        '''
        return ms * 10 ** 3 / period


class Lidar:

    DATA_PATH = /etc/data/scans
    # cartesian coordinates
    scan = [][]
    phase_angle = 0


    def __init__(self):
        self.ensure_writable()
        self.servo = Servo()
        self.scan()

    def ensure_writable(self):
        os.mkdirs(self.DATA_PATH, exist_ok=True)
        os.access(self.DATA_PATH, os.W_OK | os.X_OK)
        self.logging.info('Verified scan write permissions')

    def scan(self):
        while(self.servo.pos < 180):
            self.scan_horizon()
            self.servo.increment_deg()

    def scan_horizon(self):
        while(self.phase_angle < 180):
            # Scan point
            # record
            # rotate
            self.scan_point(self.phase_angle, self.servo.pos)
            pass

    def scan_point(self, theta, phi):
        '''
        Given angles theta and phi, convert to cartesian coords
        and record distance for said coords
        '''
        # record scan[x][y] = z
        distance = None
        # distance = get data
        x = distance * math.sin(theta) * math.cos(phi)
        y = distance * math.sin(theta) * math.sin(phi)
        z = distance * cos(theta)

        scan[x][y] = z

    def rotate_deg(deg=self.STEP_SIZE):
        self.phase_angle += deg
        # Move infetesimially
        pass

    def write(self):
        fname = 'scan-%d' % time.time()
        fpath = self.DATA_PATH + '/' + fname
        self.logging.info('Writing scan to %s...', fpath)
        pickle.dump(fpath, self.scan)
