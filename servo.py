#!/usr/bin/env python3

import logging

import RPi.GPIO as gpio


class Servo:
    PWM_FREQ= 50 # hz
    PERIOD = PWM_FREQ / 60 # 0.02 secs
    NEUTRAL = 1.5 # ms
    MIN = 500 * 10 ** -3 # in ms
    MAX = 2500 * 10 ** -3 # in ms
    DUTY_CYCLE =  MIN / PERIOD
    STEP = 3 * 10 ** -3 # in ms
    STEPS_DEG = (MAX - MIN) / 180

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

    def reset_pos(self) -> None:
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
