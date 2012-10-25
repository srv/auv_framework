# Simple implementation of a Discrete 
# Proportional-Integral-Derivative (PID) controller. 
# PID controller gives output value for error between 
# desired reference input and measurement feedback to minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller

class Pid:
    """
    Discrete PID control
    """
    def __init__(self, k_p, k_i, k_d, integral_min = -10, integral_max = 10):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.__setpoint = 0.0
        self.__integral = 0.0
        self.__previous_error = 0.0
        self.__integral_min = integral_min
        self.__integral_max = integral_max

    def update(self, feedback_value, dt):
        """
        Calculate PID output value for stored reference input (setpoint) and 
        feedback using the time passed dt.
        The integral part is limited to integral_max and integral_min
        """
        error = self.__setpoint - feedback_value
        self.__integral = self.__integral + error * dt
        if self.__integral > self.__integral_max:
            self.__integral = self.__integral_max
        if self.__integral < self.__integral_min:
            self.__integral = self.__integral_min
        if dt > 0:
            derivative = (error - self.__previous_error) / dt
        else:
            derivative = 0.0
        self.__previous_error = error
        output = self.k_p * error + self.k_i * self.__integral + self.k_d * derivative
        return output

    def setSetpoint(self, setpoint):
        """
        Initilize the setpoint of PID
        """
        epsilon = 1e-10
        if abs(setpoint - self.__setpoint) > epsilon:
            self.__setpoint = setpoint
            self.__previous_error = 0.0

    def getSetpoint(self):
        """
        Returs the current setpoint.
        """
        return self.__setpoint

