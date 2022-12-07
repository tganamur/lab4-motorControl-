from pyb import Timer
from pyb import Pin
class L6206:

    def __init__ (self, PWM_tim, IN1_pin, IN2_pin, EN_pin):

        self.PWM_tim = PWM_tim
        self.IN1_pin = IN1_pin
        self.IN2_pin = IN2_pin
        self.EN_pin = EN_pin
        self.ch1 = self.PWM_tim.channel(1,Timer.PWM_INVERTED, pin = self.IN1_pin,pulse_width_percent = 0)
        self.ch2 = self.PWM_tim.channel(2,Timer.PWM_INVERTED, pin = self.IN2_pin,pulse_width_percent = 0)
        pass

    def set_duty (self, duty):
        #print('Duty cycle set \n')
    #decides which channel gets the PWM and sets pins accordingly
        if duty >= 0:
            #telling the timer to PWM the in2 pin
            self.ch1.pulse_width_percent(100-duty)
            self.ch2.pulse_width_percent(100)

        elif duty < 0:
            #telling the timer to PWM the in1 pin
            self.ch1.pulse_width_percent(100)
            self.ch2.pulse_width_percent(100-abs(duty))

        pass

    def enable (self):
        self.EN_pin = Pin(self.EN_pin,mode=Pin.OUT_PP)
        self.EN_pin.high()
        #print('enable pin high\n')
        pass

    def disable (self):
        self.EN_pin = Pin(self.EN_pin,mode=Pin.OUT_PP)
        self.EN_pin.low()
        print('enable pin low\n')
        pass