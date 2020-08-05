import logging
import time

from modules import cbpi
from modules.core.controller import KettleController
from modules.core.props import Property
from modules.core.hardware import ActorBase

from modules.core.props import Property, StepProperty
from modules.core.step import StepBase

try:
    import RPi.GPIO as GPIO

    GPIO.setmode(GPIO.BCM)
except Exception as e:
    print()
    e
    pass

class PID(object):
    ek_1 = 0.0
    xk_1 = 0.0
    xk_2 = 0.0

    yk = 0.0

    GMA_HLIM = 100.0
    GMA_LLIM = 0.0

    def __init__(self, ts, kc, ti, td, Pmax=100.0):
        self.kc = kc
        self.ti = ti
        self.td = td
        self.ts = ts
        self.GMA_HLIM = Pmax
        self.k0 = 0.0
        self.k1 = 0.0
        self.pp = 0.0
        self.pi = 0.0
        self.pd = 0.0

        if (self.ti == 0.0):
            self.k0 = 0.0
        else:
            self.k0 = self.kc * self.ts / self.ti
        self.k1 = self.kc * self.td / self.ts

    def calc(self, xk, tset):

        ek = 0.0
        ek = tset - xk # calculate e[k] = SP[k] - PV[k]

        self.pp = self.kc * (self.xk_1 - xk) # y[k] = y[k-1] + Kc*(PV[k-1] - PV[k])
        self.pi = self.k0 * ek  # + Kc*Ts/Ti * e[k]
        self.pd = self.k1 * (2.0 * self.xk_1 - xk - self.xk_2)
        self.yk += self.pp + self.pi + self.pd
        print ("------------")
        print((self.yk, self.pp, self.pi, self.pd))

        self.xk_2 = self.xk_1  # PV[k-2] = PV[k-1]
        self.xk_1 = xk    # PV[k-1] = PV[k]

        # limit y[k] to GMA_HLIM and GMA_LLIM
        if (self.yk > self.GMA_HLIM):
            self.yk = self.GMA_HLIM
        if (self.yk < self.GMA_LLIM):
            self.yk = self.GMA_LLIM

        return round(self.yk, 1)

@cbpi.controller
class PIDHendi(KettleController):

    P = Property.Number("P", configurable=True, default_value=40, unit="")
    I = Property.Number("I", configurable=True, default_value=140, unit="")
    D = Property.Number("D", configurable=True, default_value=0, unit="")
    Pmax = Property.Number("Max Power", configurable=True, default_value=100, unit="%")

    def run(self):
        p = float(self.P)
        i = float(self.I)
        d = float(self.D)
        pmax = int(self.Pmax)
        ts = 5

        print((p, i, d, pmax))
        pid = PID(ts, p, i, d, pmax)

        while self.is_running():
            heat_percent = pid.calc(self.get_sensor_value(), self.get_target_temp())
            if heat_percent == 0:
                self.actor_power(heat_percent)
                self.heater_off()
                cbpi.log_action("PIDHendi OFF {}")
            else:
                self.actor_power(heat_percent)
                self.heater_on(power=heat_percent)

                cbpi.log_action("PIDHendi calling heater_on(power={})".format(heat_percent))

            self.sleep(ts)

        self.heater_off()

@cbpi.controller
class BoilHendi(KettleController):

    def run(self):
        ts = 5

        while self.is_running():
            #heat_percent = min(self.get_target_temp(), pmax)
            heat_percent = self.actor_power()
            print(("heat_percent = {}".format(heat_percent)))
            if heat_percent == 0:
                self.actor_power(heat_percent)
                self.heater_off()
            else:
                self.heater_on(power=heat_percent)
                self.actor_power(heat_percent)

            self.sleep(ts)

        self.heater_off()

@cbpi.actor
class HendiControl(ActorBase):
    power_pin = Property.Select("Power control GPIO",
                                options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                                         20, 21, 22, 23, 24, 25, 26, 27], )
    onoff_pin = Property.Select("On/Off control GPIO",
                                options=[0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
                                         20, 21, 22, 23, 24, 25, 26, 27])
    freq = Property.Number("PWM frequency", configurable=True)
    Pmax = Property.Number("Max Power", configurable=True, default_value=100, unit="%")

    power = 0
    pwm = None
    stopped = True

    def init(self):
        GPIO.setmode(GPIO.BCM)
        # setup pins for power control
        GPIO.setup(int(self.power_pin), GPIO.OUT)
        # setup pins for on/off control
        GPIO.setup(int(self.onoff_pin), GPIO.OUT)
        GPIO.output(int(self.onoff_pin), 0)
        HendiControl.power =  int(self.Pmax)

    def on(self, power=None):
        HendiControl.stopped = False
        if HendiControl.pwm is None:
            if HendiControl.freq is None:
                HendiControl.freq = 100
            HendiControl.pwm = GPIO.PWM(int(self.power_pin), int(self.freq))
            HendiControl.pwm.start(int(HendiControl.power))
        if(0 == HendiControl.power):
            GPIO.output(int(self.onoff_pin), 0)
        else:
            GPIO.output(int(self.onoff_pin), 1)
            HendiControl.pwm.start(1)
            HendiControl.pwm.ChangeDutyCycle(int(HendiControl.power))
            cbpi.log_action("ON, Set power {}".format(HendiControl.power))

    def set_power(self, power):
        HendiControl.power = min(int(power), int(self.Pmax))
        cbpi.log_action("Set power {}".format(HendiControl.power))
        self.pwm.ChangeDutyCycle(HendiControl.power)



    def off(self):
        cbpi.log_action("off")
        self.stopped = True
        self.pwm.ChangeDutyCycle(0)
        self.pwm.stop()
        GPIO.output(int(self.onoff_pin), 0)

@cbpi.step
class ToBoilStep(StepBase):
    '''
    Just put the decorator @cbpi.step on top of a method
    '''
    # Properties
    temp = Property.Number("Power", configurable=True)
    kettle = StepProperty.Kettle("Kettle")
    s = False

    def init(self):
        '''
        Initialize Step. This method is called once at the beginning of the step
        :return:
        '''
        # set target tep
        self.s = False
        print(("ToBoilStep init: {}".format(int(self.temp))))
        self.set_target_temp(self.temp, self.kettle)

    def reset(self):
        self.stop_timer()
        self.set_target_temp(self.temp, self.kettle)

    def finish(self):
        self.set_target_temp(0, self.kettle)

    def execute(self):
        '''
        This method is execute in an interval
        :return:
        '''

        # Check if Target Temp is reached
        #if self.get_kettle_temp(self.kettle) >= int(self.temp) and self.s is False:
        #    self.s = True
        #    self.notify("Step Temp Reached!", "Please press the next button to continue", timeout=None)
        #self.actor_power(self.get_target_temp)
        pass

@cbpi.step
class BoilStep(StepBase):
    '''
    Just put the decorator @cbpi.step on top of a method
    '''
    # Properties
    power = Property.Number("Power", configurable=True)
    kettle = StepProperty.Kettle("Kettle")
    timer = Property.Number("Timer in Minutes", configurable=True)

    def init(self):
        '''
        Initialize Step. This method is called once at the beginning of the step
        :return:
        '''
        # set target tep
        print(("class BoilStep(StepBase): power = {}".format(self.kettle)))
        self.actor_power(1, self.power)
        #self.set_target_temp(self.power, self.kettle)

    @cbpi.action("Start Timer Now")
    def start(self):
        '''
        Custom Action which can be execute form the brewing dashboard.
        All method with decorator @cbpi.action("YOUR CUSTOM NAME") will be available in the user interface
        :return:
        '''
        if self.is_timer_finished() is None:
            self.start_timer(int(self.timer) * 60)

    def reset(self):
        self.stop_timer()
        self.actor_power(0, self.power)
        #self.set_target_temp(self.temp, self.kettle)

    def finish(self):
        self.actor_power(0, 0)
        self.set_target_temp(0, self.kettle)

    def execute(self):
        '''
        This method is execute in an interval
        :return:
        '''

        # Check if Timer is Running
        if self.is_timer_finished() is None:
            self.start_timer(int(self.timer) * 60)

        # Check if timer finished and go to next step
        if self.is_timer_finished() == True:
            next(self)
