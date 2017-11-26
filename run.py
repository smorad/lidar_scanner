#!/usr/bin/env python3

# Inits hardware PWM server and begins laser scan

import scan
import servo
import time

with servo.Pigpiod() as daemon:
    print('Awaiting pigiod init...')
    time.sleep(1)
    l = scan.Lidar()
    l.scan()

print('Done')
