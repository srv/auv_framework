# Simple implementation of a Discrete 
# Proportional-Integral-Derivative (PID) controller. 
# PID controller gives output value for error between 
# desired reference input and measurement feedback to minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller

class Pid:
    """
    Discrete PID control
    """
    def __init__(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.setSetpoint(0.0)

    def update(self, feedback_value, dt):
        """
        Calculate PID output value for stored reference input (setpoint) and 
        feedback using the time passed dt.
        """
        error = self.__setpoint - feedback_value
        self.__integral = self.__integral + error * dt
        derivative = (error - self.__previous_error) / dt
        output = self.k_p * error + self.k_i * self.__integral + self.k_d * derivative
        self.__previous_error = error

    def setSetpoint(self, setpoint):
        """
        Initilize the setpoint of PID
        """
        self.__setpoint = setpoint
        self.__integral = 0.0
        self.__previous_error = 0.0
