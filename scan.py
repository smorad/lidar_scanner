#!/usr/bin/env python3

import time
import math
import pickle
import os
import logging

import RPi.GPIO as gpio

# Load .so for correct arch
if os.uname.machine == 'x86_64':
    import amd64.hokuyoaist as lidar
else:
    import arm.hokuyoaist as lidar


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
    scan = {}{}
    dev_path = '/dev/ttyACM0'
    phase_angle = 0

    def sys_init(self):
        '''
        Lidar system init
        '''
        try:
            sensor = lidar.sensor.open(self.dev_path)
            self.STEP_SIZE_DEG = sensor.step_to_angle()

    def __init__(self):
        self.ensure_writable()
        self.servo = Servo()
        self.sys_init()
        self.scan()

    def ensure_writable(self):
        os.mkdirs(self.DATA_PATH, exist_ok=True)
        os.access(self.DATA_PATH, os.W_OK | os.X_OK)
        self.logging.info('Verified scan write permissions')

    def scan(self):
        while(self.servo.pos < 180):
            logging.info('Scanning horizon at phi = %f deg', self.servo.pos)
            self.scan_horizon()
            self.servo.increment_deg()
        self.servo.reset_pos()

    def to_cartesian(theta: float, phi: float, r: float) -> (float, float, float):
        '''
        Convert sphereical to cartesian coords
        '''
        x = r * math.sin(theta) * math.cos(phi)
        y = r * math.sin(theta) * math.sin(phi)
        z = r * cos(theta)
        return (x, y, z)

    def scan_horizon(self, theta_0: float, theta_1: float, phi: float):
        '''
        Scans from theta_0 to theta_1 in deg, then saves the resulting data in
        cartesian form
        '''
        sensor = lidar.Sensor()
        r_values = lidar.ScanData()
        try:
            sensor.open(self.dev_path)
            num_samples = sensor.get_new_ranges_by_angle(r_values, theta_0, theta_1)
        finally:
            sensor.close()

        for idx in range(num_samples):
            # Angular resolution per sample
            theta = idx / (theta_1 - theta_0)
            r = r_values.range(idx)
            x, y, z = self.to_cartesian(theta, phi, r)
            # Save data for writing
            scan[x][y] = z
        
        r_values.clean_up()

    def rotate_deg(deg=self.STEP_SIZE_DEG):
        # Move infetesimially
        self.phase_angle += deg

    def write(self):
        fname = 'scan-%d' % time.time()
        fpath = self.DATA_PATH + '/' + fname
        self.logging.info('Writing scan to %s...', fpath)
        pickle.dump(fpath, self.scan)
