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
    scan_data = []
    port_opts = 'type=serial,device=/dev/ttyACM0,timeout=1'

    def sys_init(self):
        '''
        Lidar system init
        '''
        try:
            sensor = lidar.Sensor()
            sensor.open(self.port_opts)
            self.STEP_SIZE_DEG = sensor.step_to_angle(1)
        finally:
            sensor.close()

    def __init__(self):
        logging.basicConfig(level=logging.DEBUG)
        self.ensure_writable()
        self.servo = servo.Servo()
        self.sys_init()

    def ensure_writable(self):
        os.makedirs(self.DATA_PATH, exist_ok=True)
        os.access(self.DATA_PATH, os.W_OK | os.X_OK)
        logging.info('Verified scan write permissions')

    def scan(self):
        # turns out the maximum angle is 89.xxx
        while(self.servo.phase_angle < 89):
                logging.info(
                    'Scanning horizon at phi: {:.2f} deg'.format(self.servo.phase_angle))
                self.scan_horizon(self.servo.phase_angle)
                self.servo.increment()

        self.write()
        self.draw_pointmap(self.scan_data)
        self.servo.reset_pos()

    def to_cartesian(self, r: float, theta_deg: float, phi_deg: float) -> (float, float, float):
        '''
        Convert sphereical deg to cartesian coords
        '''
        theta, phi = map(math.radians, [theta_deg, phi_deg])
        # xy plane is the floor
        # z plane going up
        # phi is the angle from the Z-axis to vector r
        x = r * math.sin(phi) * math.cos(theta)
        y = r * math.sin(phi) * math.sin(theta)
        z = r * math.cos(phi)
        return (x, y, z)

    def scan_horizon(self, phi: float):
        '''
        Scans from theta_0 to theta_1 in rad, then saves the resulting data in
        cartesian form. The hardware is limited to arcs of < 2 rad
        '''
        # according to docs:
        # 683 steps
        # scan ccw from top
        step_arc = 360 / 1024 # in deg, from spec sheet
        sensor = lidar.Sensor()
        r_values = lidar.ScanData()
        try:
            sensor.open(self.port_opts)
            num_samples = sensor.get_new_ranges(r_values)
        finally:
            sensor.close()

        for idx in range(r_values.ranges_length()):
            # Angular resolution per sample
            theta = idx * step_arc
            r = r_values.range(idx)
            # r less than 20mm means the sample is bad
            if r <= 20:
                logging.info(
                    'Got bad r at theta: {:.2f} phi: {:.2f}'.format(theta, phi))
                continue
            x, y, z = self.to_cartesian(r, theta, phi)
            # We don't want to overwrite any data
            #assert(x not in self.scan_data)
            # Save data for writing
            self.scan_data.append((x, y, z))
        
        r_values.clean_up()
        return self.scan_data

    def write(self):
        fname = 'scan-%d' % time.time()
        fpath = self.DATA_PATH + '/' + fname
        logging.info('Writing scan to %s...', fpath)
        with open(fpath, 'wb') as f:
            pickle.dump(self.scan_data, f)

    def draw_pointmap(self, data):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from mpl_toolkits.mplot3d import Axes3D
        plt.ioff()
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel('X in mm')
        ax.set_ylabel('Y in mm')
        ax.set_zlabel('Z in mm')

        x = [a[0] for a in data]
        y = [a[1] for a in data]
        z = [a[2] for a in data]
        # draw our position
        ax.scatter([0], [0], [0], c='b', marker='^')
        ax.scatter(x, y, z, c='r', marker='.')
        
        plt.savefig('figure')
