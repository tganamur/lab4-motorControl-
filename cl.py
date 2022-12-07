class cl: 
    def __init__(self):
        self.kp = 1
        self.set_pt = 0

    def p_eff(self, measurement):
        eff = (self.set_pt - measurement)*self.kp
        return eff

    def pi(self,measurement):
        err = self.set_pt - measurement
        self.buildup += err
        eff = err*self.kp + self.buildup*self.ki
        return eff

    def set_kp(self, kp):
        self.kp = kp
    
    def set_velTarget(self, set_pt):
        self.set_pt = set_pt