
class PIDCurrentRegulator:
    def __init__(self, Kp, Ki, Kd, Ki_min, Ki_max):
        # Define the PID gains
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.Ki_min = Ki_min
        self.Ki_max = Ki_max

        # Define the variables
        self._last_error = 0
        self._integral = 0


    def regulate_current(self, setpoint_current_A, measured_current_A):
        # Calculate the error
        error = setpoint_current_A - measured_current_A

        # Calculate the integral
        self._integral = self._integral + error

        if self._integral > self.Ki_max:
            self._integral = self.Ki_max
        elif self._integral < self.Ki_min:
            self._integral = self.Ki_min

        # Calculate the derivative
        derivative = error - self._last_error

        # Calculate the output voltage
        output_voltage = self.Kp * error + self.Ki * self._integral + self.Kd * derivative

        # Save the last error
        self._last_error = error

        return output_voltage
    
