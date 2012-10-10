# Simple implementation of a Discrete 
# Proportional-Integral-Derivative (PID) controller. 
# PID controller gives output value for error between 
# desired reference input and measurement feedback to minimize error value.
# More information: http://en.wikipedia.org/wiki/PID_controller
#
#######    Example    #########
#
# pid = Pid(1.5, 0.2, 0.0, -10.0, 10.0)
# pid.setSetPoint(5.0)
# while True:
#     # get your measurement
#     output = pid.update(measurement_value)

def clamp(value, min_value, max_value):
    if value > max_value:
        return max_value
    if value < min_value:
        return min_value
    return value

class Pid:
    """
    Discrete PID control
    """
    def __init__(self, k_p, k_i, k_d, max_output, min_output):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.max_output = max_output
        self.min_output = min_output
        self.setSetpoint(0.0)

    def update(self, feedback_value, dt):
        """
        Calculate PID output value for stored reference input (setpoint) and 
        feedback using the time passed dt.
        """
        error = self.setpoint - feedback_value
        self.integral = self.integral + error * dt
        derivative = (error - self.previous_error) / dt
        output = self.k_p * error + self.k_i * self.integral + self.k_d * derivative

        self.previous_error = error
        return clamp(output, self.min_output, self.max_output)

    def setSetpoint(self, setpoint):
        """
        Initilize the setpoint of PID
        """
        self.setpoint = setpoint
        self.integral = 0.0
        self.previous_error = 0.0

    def setConstants(self, k_p, k_i, k_d):
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d

