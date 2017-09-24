#!/usr/bin/env python3

import time
import math
import pickle
import os
import logging

import servo

# Load .so for correct arch
if os.uname().machine == 'x86_64':
    import amd64.hokuyoaist as lidar
else:
    import arm.hokuyoaist as lidar


class Lidar:

    DATA_PATH = '/etc/data/scans'
    # cartesian coordinates
    scan = {}
    dev_path = '/dev/ttyACM0'

    def sys_init(self):
        '''
        Lidar system init
        '''
        try:
            sensor = lidar.sensor.open(self.dev_path)
            self.STEP_SIZE_DEG = sensor.step_to_angle()
        finally:
            sensor.close()

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
            # We don't want to overwrite any data
            assert(x not in scan)
            # Save data for writing
            scan[x] = {y: z}
        
        r_values.clean_up()

    def write(self):
        fname = 'scan-%d' % time.time()
        fpath = self.DATA_PATH + '/' + fname
        self.logging.info('Writing scan to %s...', fpath)
        pickle.dump(fpath, self.scan)
