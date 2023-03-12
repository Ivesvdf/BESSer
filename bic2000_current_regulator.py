import skfuzzy as fuzz
from skfuzzy import control as ctrl
import numpy as np

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
    
class FuzzyCurrentRegulator:
    def __init__(self, voltage_range=(-10, 10), current_range=(-5, 5)):
        self.voltage_range = voltage_range
        self.current_range = current_range

        self.Ki = 0
        self.Kd = 0
        self.Kp = 0
        self._integral = 0 
        
        # Create input and output variables
        self.current_error = ctrl.Antecedent(np.arange(*current_range, 0.01), 'current_error')
        self.voltage_adjustment = ctrl.Consequent(np.arange(*voltage_range, 0.01), 'voltage_adjustment')

        # Define fuzzy sets for input variable
       # self.current_error['negative'] = fuzz.trimf(self.current_error.universe, [current_range[0], current_range[0], 0])
        #self.current_error['zero'] = fuzz.trimf(self.current_error.universe, [-0.1, 0, 0.1])
        #self.current_error['positive'] = fuzz.trimf(self.current_error.universe, [0, current_range[1], current_range[1]])
        self.current_error.automf(3, names=['negative', 'zero', 'positive'])
       # self.current_error.view()

        # Define fuzzy sets for output variable
        #self.voltage_adjustment['negative'] = fuzz.trimf(self.voltage_adjustment.universe, [voltage_range[0], voltage_range[0], 0 ])
        #self.voltage_adjustment['zero'] = fuzz.trimf(self.voltage_adjustment.universe, [-0.1, 0, 0.1])
       # self.voltage_adjustment['positive'] = fuzz.trimf(self.voltage_adjustment.universe, [0, voltage_range[1], voltage_range[1]])
        self.voltage_adjustment.automf(3, names=['negative', 'zero', 'positive'])

        # Define fuzzy rules
        rule1 = ctrl.Rule(self.current_error['negative'], self.voltage_adjustment['negative'])
        rule2 = ctrl.Rule(self.current_error['zero'], self.voltage_adjustment['zero'])
        rule3 = ctrl.Rule(self.current_error['positive'], self.voltage_adjustment['positive'])

        # Create fuzzy control system
        self.control_system = ctrl.ControlSystem([rule1, rule2, rule3])
        # self.control_system.view()

        # Create simulation
        self.control_simulation = ctrl.ControlSystemSimulation(self.control_system)

    def regulate_current(self, setpoint_current_A, measured_current_A):
        # Calculate current error
        current_error = setpoint_current_A - measured_current_A

        # Set input values and compute output
        self.control_simulation.input['current_error'] = current_error
        self.control_simulation.compute()

        # Get output value and return it
        voltage_delta = self.control_simulation.output['voltage_adjustment']

        b = False
        if b:
            self.current_error.view(sim=self.control_simulation)
        return voltage_delta