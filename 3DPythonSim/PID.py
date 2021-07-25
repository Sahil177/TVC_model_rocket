
def clamp(val, upper, lower):
    if val > upper:
        return upper
    elif  val < lower:
        return lower
    else:
        return val



class PID:
    def __init__(self, kp, ki, kd, tau, sampletime, outputlim, setpoint = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.st = sampletime
        self.tau = tau
        self.outputlim = outputlim
        self.setpoint = setpoint
        self.prev_error_x = 0
        self.prev_error_y = 0
        self.I_x = 0
        self.I_y = 0
        self.D_x = 0
        self.D_y = 0
    
    def PID_compute(self, theta, last_error,last_I, last_D):
        #compute error
        error = theta - self.setpoint
        #compute proportional term
        P = self.kp*error
        #compute integral term
        I = last_I + self.ki*((error+last_error)/2)*self.st
        #compute derivative term
        D = (2*self.kd*(error - last_error) + (2*self.tau-self.st)*last_D)/(2*self.tau+self.st)
        #compute output signal
        phi = P + I + D
        phi = clamp(phi, self.outputlim, -self.outputlim)
        return phi, error, I, D


