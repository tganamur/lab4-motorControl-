from pyb import Timer
class Encoder:
    '''!@brief Interface with quadrature encoders
    @details
    '''

    def __init__(self,timer,pin1,pin2):
    
        self.position = 0
        self.AR = 65535
        self.timer = timer
        self.pin1 = pin1
        self.pin2 = pin2
        self.ch1 = self.timer.channel(1,Timer.ENC_AB,pin = pin1)
        self.ch2 = self.timer.channel(2,Timer.ENC_AB,pin = pin2)
        self.delta = 0
        self.temp = 0
        self.RAR = int((self.AR+1)/2)
    #print('Creating encoder object')

    def update(self):
        self.delta = self.timer.counter() - self.temp
        
        if(self.delta > self.RAR):
            self.delta = self.delta - (self.AR + 1)
        elif(self.delta < -self.RAR):
            self.delta = self.delta + (self.AR + 1)
        
        self.temp = self.timer.counter()
        self.position = self.delta + self.position
        '''!@brief Updates encoder position and delta
        @details
        '''
    #print('Reading encoder count and updating position and delta values')

    def zero(self):
        self.position = 0
        '''!@brief Resets the encoder position to zero
        @details
        '''
    #print('Setting position back to zero')
