import threading
import time
from bic2000_current_regulator import PIDCurrentRegulator
import threadsafe_can
from loguru import logger
import enum


class BICCommand(enum.Enum):
    OPERATION = 0x0
    VOUT_SET = 0x20
    IOUT_SET = 0x30
    FAULT_STATUS = 0x40
    READ_VIN = 0x50
    READ_VOUT = 0x60
    READ_IOUT = 0x61
    READ_TEMPERATURE_1 = 0x62
    MFR_ID_B0B5 = 0x80
    MFR_ID_B6B11 = 0x81
    MFR_MODEL_B0B5 = 0x82
    MFR_MODEL_B6B11 = 0x83
    MFR_REVISION_B0B5 = 0x84
    MFR_LOCATION_B0B2 = 0x85

    SCALING_FACTOR = 0x00C0
    SYSTEM_STATUS = 0x00C1
    SYSTEM_CONFIG = 0x00C2
    DIRECTION_CTRL = 0x0100
    REVERSE_VOUT_SET = 0x0120
    REVERSE_IOUT_SET = 0x0130
    BIDIRECTIONAL_CONFIG = 0x0140

class FaultStatusFlag(enum.Enum):
    FAN_LOCKED = 1 << 0  # Fan locked flag (0 = Fan working normally, 1 = Fan locked)
    OVER_TEMP = 1 << 1  # Over temperature protection (0 = Internal temperature normal, 1 = Internal temperature abnormal)
    DC_OVER_VOLTAGE = 1 << 2  # DC over voltage protection (0 = DC voltage normal, 1 = DC over voltage protected)
    DC_OVER_CURRENT = 1 << 3  # DC overcurrent protection (0 = DC current normal, 1 = DC over current protected)
    SHORT_CIRCUIT = 1 << 4  # Short circuit protection (0 = Shorted circuit does not exist, 1 = Shorted circuit protected)
    AC_ABNORMAL = 1 << 5  # AC abnormal flag (0 = AC range normal, 1 = AC range abnormal)
    DC_TURNED_OFF = 1 << 6  # DC status (0 = DC turned on, 1 = DC turned off)
    INTERNAL_TEMP_ABNORMAL = 1 << 7  # Internal high temperature protection (0 = Internal temperature normal, 1 = Internal temperature abnormal)
    HV_OVER_VOLTAGE = 1 << 8  # HV over voltage protection (0 = HV voltage normal, 1 = HV over voltage protected)

class SystemStatusFlags(enum.Enum):
    PARALLEL_MODE_STATUS = 1 << 0  # Parallel mode status (0 = Current device is Slave, 1 = Current device is Master)
    SECONDARY_DD_OUTPUT_VOLTAGE_STATUS = 1 << 1  # Secondary DD output voltage status (0 = Secondary DD output voltage status TOO LOW, 1 = Secondary DD output voltage status NORMAL)
    PRIMARY_PFC_STATUS = 1 << 2  # Primary PFC status (0 = Primary PFC OFF or abnormal, 1 = Primary PFC ON normally)
    ACTIVE_DUMMY_LOAD_CONTROL_STATUS = 1 << 4  # Active dummy load control status (0 = Active dummy load off/function not supported, 1 = Active dummy load on)
    DEVICE_INITIALIZED_STATUS = 1 << 5  # Device initialized status (0 = In initialization status, 1 = NOT in initialization status)
    EEPROM_DATA_ACCESS_ERROR = 1 << 6  # EEPROM data access error (0 = EEPROM data access normal, 1 = EEPROM data access error)



class SystemConfig:
    """
    Represents the CANBus communication control status and the pre-set value of
    power on operation command as a class.
    """
    class CANCtrl(enum.Enum):
        """
        Represents the bit flags for the CANBus communication control status.
        """
        DISABLED = 0
        ENABLED = 1

        def __str__(self):
            return self.name

    class OperationInit(enum.Enum):
        """
        Represents the pre-set value of power on operation command.
        """
        POWER_OFF = 0b00
        POWER_ON = 0b01
        PREVIOUS_SET_VALUE = 0b10

        def __str__(self):
            return self.name
    
    def __init__(self, value: int):
        self.can_ctrl = SystemConfig.CANCtrl(value & 0b1)
        self.operation_init = SystemConfig.OperationInit((value >> 1) & 0b11)

    def __str__(self):
        return f"CAN_CTRL: {self.can_ctrl}, OPERATION_INIT: {self.operation_init}"

def parse_factor(value: int) -> float:
    if value == 0x0:
        return None
    elif value == 0x4:
        return 0.001
    elif value == 0x5:
        return 0.01
    elif value == 0x6:
        return 0.1
    elif value == 0x7:
        return 1.0
    elif value == 0x8:
        return 10.0
    elif value == 0x9:
        return 100.0
    else:
        return None
    
def get_vout_adjustable_range(model_voltage_V):
    # Define the VOUT_SET portion of the table as a list of dictionaries, one per row
    vout_table = [
        {"model": 12, "adjustable range": (10, 15), "tolerance": 0.12, "default": 12},
        {"model": 24, "adjustable range": (19, 28), "tolerance": 0.24, "default": 24},
        {"model": 48, "adjustable range": (38, 65), "tolerance": 0.48, "default": 48},
        {"model": 96, "adjustable range": (76, 112), "tolerance": 0.96, "default": 96},
    ]
    # Find the row corresponding to the input model
    for row in vout_table:
        if row["model"] == model_voltage_V:
            return row["adjustable range"]
    return (0,0)

def get_reverse_vout_adjustable_range(model_voltage_V):
    # Define the REVERSE_VOUT_SET portion of the table as a list of dictionaries, one per row
    reverse_vout_table = [
        {"model": 12, "adjustable range": (-15, -10), "tolerance": 0.12, "default": -12},
        {"model": 24, "adjustable range": (-28, -19), "tolerance": 0.24, "default": -24},
        {"model": 48, "adjustable range": (-65, -38), "tolerance": 0.48, "default": -48},
        {"model": 96, "adjustable range": (-112, -76), "tolerance": 0.96, "default": -96},
    ]
    # Find the row corresponding to the input model
    for row in reverse_vout_table:
        if row["model"] == model_voltage_V:
            return row["adjustable range"]
    return (0,0)

def get_iout_adjustable_range(model_voltage_V):
    # Define the REVERSE_VOUT_SET portion of the table as a list of dictionaries, one per row
    iout_table = [
        {"model": 12, "adjustable range": (36, 198), "tolerance": 4, "default": 198},
        {"model": 24, "adjustable range": (18, 99), "tolerance": 2, "default": 99},
        {"model": 48, "adjustable range": (9, 49.5), "tolerance": 1, "default": 49.5},
        {"model": 96, "adjustable range": (4.5, 24.75), "tolerance": 0.5, "default": 24.75},
    ]
    # Find the row corresponding to the input model
    for row in iout_table:
        if row["model"] == model_voltage_V:
            return row["adjustable range"]
    return (0,0)

def get_reverse_iout_adjustable_range(model_voltage_V):
    # Define the REVERSE_VOUT_SET portion of the table as a list of dictionaries, one per row
    reverse_iout_table = [
        {"model": 12, "adjustable range": (-153,-36), "tolerance": 4, "default": -153},
        {"model": 24, "adjustable range": (-76.5,-18), "tolerance": 2, "default": -76.5},
        {"model": 48, "adjustable range": (-38.3,-9), "tolerance": 1, "default": -38.3},
        {"model": 96, "adjustable range": (-19.1, -4.5), "tolerance": 0.5, "default": -19.1},
    ]
    # Find the row corresponding to the input model
    for row in reverse_iout_table:
        if row["model"] == model_voltage_V:
            return row["adjustable range"]
    return (0,0)

def parse_flags(the_enum, value):
    """Receives an integer representing the device status flags and returns a dictionary containing the status of each flag"""
    flags = set()
    for flag in the_enum:
        if value & flag.value:
            flags.add(flag)
    return flags

def from_twos_complement(num: int, bits: int) -> int:
    """Converts a two's complement integer to a signed integer"""
    if num & (1 << (bits - 1)):
        # Negative number
        inverted = num ^ ((1 << bits) - 1)
        return -(inverted + 1)
    else:
        # Positive number
        return num
def to_twos_complement(num: int, bits: int) -> int:
    """Returns the two's complement integer representation of num with bits number of bits"""
    if num >= 0:
        # Positive numbers
        return num
    else:
        # Negative numbers
        max_val = 1 << bits
        return max_val + num

class BICChargerInverter:
    def __init__(self, config):
        self.__canbus = threadsafe_can.ThreadSafeCanInterface(threadsafe_can.Bus(**config.get("charger_inverter",dict()).get("canbus")))
        self.__canbus.add_receive_hook(self.__on_can_receive)

        device_id = config['charger_inverter']['device_id']
        if device_id > 7 or device_id < 0:
            logger.error("Invalid device id, device id should range from 0 to 7")
            device_id = 0

        self.__response_arb_id = 0x000C0200 | device_id
        self.__command_arb_id = 0x000C0300 | device_id
        self.__receive_broadcast_arb_id = 0x000C03FF

        self.__last_command_time = 0
        self.__command_interval_s = 0.025
        self.__num_repeats = 3
        self.__read_state = {}
        self.__write_state = {}

        self.__Vout_factor = None
        self.__Iout_factor = None
        self.__Vin_factor = None
        self.__temperature_1_factor = None

        self.__Vout_V = None
        self.__Vout_rest_V = None
        self.__temperature_1 = None
        self.__Vin_V = None
        self.__Iout_A = None

        self.__fault_flags = set()
        self.__system_status = set()

        self.__last_cycle_time_basic_s = 0

        self.__operational = False 
        self.__requested_charge_power_W = 0
        self.__can_control = None

        self.__invert_power_limit_W = -1725
        self.__charge_power_limit_W = 2160

        self.__deviation_counter_Ws = 0
        self.__last_deviation_update_time = time.time()

        self.__battery_voltage_limits = (config['battery']['min_voltage_V'], config['battery']['max_voltage_V'])

        model_voltage_V = config['charger_inverter']['model_voltage_V']
        self.__Iout_limits = get_iout_adjustable_range(model_voltage_V)
        self.__reverse_Iout_limits = get_reverse_iout_adjustable_range(model_voltage_V)

        current_mode = 0
        if current_mode == 0:
            self.__current_regulator = PIDCurrentRegulator(**config['charger_inverter']['PID'])

        self.__communication_thread = threading.Thread(target=self.__run, daemon=True)
        self.__receive_stop_event = threading.Event()
        self.__communication_thread.start()

    def get_last_cycle_time_basic_s(self):
        return self.__last_cycle_time_basic_s
    
    def get_charge_power_limit_W(self):
        return self.__charge_power_limit_W
    
    def get_invert_power_limit_W(self):
        return self.__invert_power_limit_W
    
    def set_battery_voltage_limits(self, limits):
        self.__battery_voltage_limits = limits

    def set_PID_Ki(self, val):
        self.__current_regulator.Ki = val
    def set_PID_Kp(self, val):
        self.__current_regulator.Kp = val
    def set_PID_Kd(self, val):
        self.__current_regulator.Kd = val

    def get_Vout_V(self):
        return self.__Vout_V

    def get_temperature_1(self):
        return self.__temperature_1

    def get_Vin_V(self):
        return self.__Vin_V

    def get_Iout_A(self):
        return self.__Iout_A

    def get_fault_flags(self):
        return set(self.__fault_flags)
    
    def get_system_status(self):
        return set(self.__system_status)

    def _calculate_setpoint(self, target_current_A):
        (min_batt_voltage_V, max_batt_voltage_V) = self.__battery_voltage_limits

        if target_current_A > 0: 
            # charge
            lower_limit_A, upper_limit_A = self.__Iout_limits
            extreme_voltage_V = max_batt_voltage_V
        else:
            # discharge
            lower_limit_A, upper_limit_A = self.__reverse_Iout_limits
            extreme_voltage_V = min_batt_voltage_V
        

        if lower_limit_A <= target_current_A <= upper_limit_A:
            # within the current control range, use constant current
            return target_current_A, extreme_voltage_V
        else:
            # outside of constant current range, use voltage control 
            target_V = self.__Vout_rest_V + self.__current_regulator.regulate_current(target_current_A, self.__Iout_A)
            target_A = 0

            # Respect limits
            if target_V > max_batt_voltage_V:
                target_V = max_batt_voltage_V
            elif target_V < min_batt_voltage_V:
                target_V = min_batt_voltage_V

            return target_A, target_V
    
    def __run(self):
        logger.info("Waiting for scaling factors")
        self.request_until_okay(lambda x: self.__start_read_command(BICCommand.SCALING_FACTOR), 
                                lambda x: self.__Vout_factor != None and not self.__receive_stop_event.is_set())
        logger.info("Scale factors received")

        logger.info("Waiting for CAN control data")
        self.request_until_okay(lambda x: self.__start_read_command(BICCommand.SYSTEM_CONFIG), 
                                lambda x: self.__can_control != None and not self.__receive_stop_event.is_set())
        logger.info("CAN Control data received")

        if not self.__can_control: 
            self.configure_bic()
            
            while not self.__receive_stop_event.is_set():
                logger.error("Your BIC had to be reprogrammed for CAN communication. Please power off and on your inverter/charger and restart this software.")
                time.sleep(1)

        logger.info("Disabling operation before starting loop.")
        self.__write_command(BICCommand.OPERATION, 0, 1)

        logger.info("Starting main communication loop")

        while not self.__receive_stop_event.is_set():
            cycle_time_start = time.time()
            self.__start_read_command(BICCommand.READ_IOUT)
            self.__start_read_command(BICCommand.READ_TEMPERATURE_1)
            self.__start_read_command(BICCommand.READ_VIN)
            self.__start_read_command(BICCommand.READ_VOUT)
            self.__start_read_command(BICCommand.OPERATION)
            self.__start_read_command(BICCommand.FAULT_STATUS)
            self.__start_read_command(BICCommand.SYSTEM_STATUS)

            if self.__Vout_V == None or self.__Iout_A == None or self.__Vin_V == None or self.__temperature_1 == None:
                logger.info("Still starting, will not execute")
            else:
                if len(self.__fault_flags) > 0:
                    operation_requested = 0

                self.__requested_current_A = self.__requested_charge_power_W / self.__Vout_V

                (instructed_current_A, target_voltage_V) = self._calculate_setpoint(self.__requested_current_A)

                logger.info(f"Will request charger/inverter for charge current {instructed_current_A} with target voltage {target_voltage_V}")

                operation_requested = self.__requested_charge_power_W != 0
                if operation_requested != self.__operational:
                    self.__write_command(BICCommand.OPERATION, 1 if operation_requested else 0, 1)

                if operation_requested:
                    if self.__requested_current_A > 0: # charge
                        self.__write_command(BICCommand.DIRECTION_CTRL, 0, 1)
                        self.__write_command(BICCommand.VOUT_SET, (int)(target_voltage_V / self.__Vout_factor), 2)
                        self.__write_command(BICCommand.IOUT_SET, (int)(instructed_current_A / self.__Iout_factor), 2)
                    else: # discharge
                        self.__write_command(BICCommand.DIRECTION_CTRL, 1, 1)
                        self.__write_command(BICCommand.REVERSE_VOUT_SET, (int)(target_voltage_V / self.__Vout_factor), 2)
                        self.__write_command(BICCommand.REVERSE_IOUT_SET, to_twos_complement((int)(instructed_current_A / self.__Iout_factor), 16), 2)
                else:
                    self.__write_command(BICCommand.IOUT_SET, 0, 2)
                    self.__write_command(BICCommand.REVERSE_IOUT_SET, 0, 2)
            
                self._update_deviation()

            self.__last_cycle_time_basic_s = time.time() - cycle_time_start

            time.sleep(0.1)

    def _update_deviation(self):
        deviation_W = abs(self.__requested_charge_power_W - self.__Vout_V*self.__Iout_A)
        now = time.time()
        deviation_Ws = deviation_W*(now - self.__last_deviation_update_time)
        self.__deviation_counter_Ws += deviation_Ws
        self.__last_deviation_update_time = now 
        
    def get_mqtt_subscriptions(self):
        return [
            ("charger_inverter_PID_Ki", float, self.set_PID_Ki),
            ("charger_inverter_PID_Kp", float, self.set_PID_Kp),
            ("charger_inverter_PID_Kd", float, self.set_PID_Kd),
        ]
    
    def get_debug_info(self):
        def fix_dict(d):
            return {str(k): v for k, v in d.items()}
        return {
            "out": fix_dict(self.__write_state),
            "in": fix_dict(self.__read_state),
            "last_cycle_time": self.get_last_cycle_time_basic_s(),
            "deviation_counter_Ws": self.__deviation_counter_Ws,
            "PID": {
                "Ki": self.__current_regulator.Ki,
                "Kd": self.__current_regulator.Kd,
                "Kp": self.__current_regulator.Kp,
                "integral": self.__current_regulator._integral
            }
        }
    
    def request_until_okay(self, command, exit_condition):
        last_scale_request_time = 0
        while not exit_condition(last_scale_request_time): 
            now = time.time()
            if now - last_scale_request_time > 1:
                last_scale_request_time = now
                command(last_scale_request_time)

    def __del__(self):
        self.stop_thread()
    
    def stop_thread(self):
        self.__receive_stop_event.set()
        self.__communication_thread.join()

    def __on_response_received(self, command:BICCommand, data:int): 
        self.__read_state[command] = data
        if command == BICCommand.FAULT_STATUS:
            self.__fault_flags = parse_flags(FaultStatusFlag, data)
        elif command == BICCommand.SYSTEM_STATUS:
            self.__system_status = parse_flags(SystemStatusFlags, data)
        elif command == BICCommand.SYSTEM_CONFIG:
            self.__can_control = (data & 1) != 0

        elif command == BICCommand.SCALING_FACTOR:
            self.__Vout_factor = parse_factor(data & 0x0F)
            self.__Iout_factor = parse_factor((data & 0xF0) >> 4)
            self.__Vin_factor = parse_factor((data & 0x0F00) >> 8)
            self.__temperature_1_factor = parse_factor((data & 0x0F0000) >> 16)
        elif command == BICCommand.OPERATION:
            self.__operational = (data == 1)
        elif command == BICCommand.READ_VOUT:
            self.__Vout_V = data * self.__Vout_factor

            if self.__operational == 0:
                self.__Vout_rest_V = self.__Vout_V
        elif command == BICCommand.READ_TEMPERATURE_1:
            self.__temperature_1 = from_twos_complement(data, 16) * self.__temperature_1_factor # todo support negative temperatures!
        elif command == BICCommand.READ_VIN:
            self.__Vin_V = data * self.__Vin_factor
        elif command == BICCommand.READ_IOUT:
            self.__Iout_A = from_twos_complement(data, 16) * self.__Iout_factor


    def __on_can_receive(self, msg: threadsafe_can.Message):
        # logger.info(f"Received {msg}")

        try:
            if msg.arbitration_id == self.__receive_broadcast_arb_id or msg.arbitration_id == self.__response_arb_id:
                data = msg.data
                
                # pad with zeros until length is 6
                while len(data) < 6:
                    data.append(0)

                try:
                    command = BICCommand(data[0] | (data[1]<<8))
                    data = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
                except ValueError:
                    logger.error(f"Invalid response received: {data}")

                self.__on_response_received(command, data)
        except:
            logger.exception("Unknown BIC response received")

    
    def request_charge_discharge(self, charge_power_W: float): 
        """Request power to be put into the battery (if argument is positive),
        or drawn from the battery by the inverter (if argument is negative)."""
        
        self.__requested_charge_power_W = charge_power_W
    
    def __write_command(self, command:BICCommand, value:int, num_data_bytes:int): 
        self.__write_state[command] = value

        for _ in range(self.__num_repeats):
            data = [ command.value & 0xFF, (command.value & 0xFF00) >> 8, (value & 0xFF), (value & 0xFF00) >> 8 ]
            self.__canbus.send(threadsafe_can.Message(arbitration_id=self.__command_arb_id, dlc=2+num_data_bytes, data=data, is_extended_id=True))

            now = time.time()
            if now - self.__last_command_time < self.__command_interval_s: 
                time.sleep(now - self.__last_command_time)

        self.__last_command_time = time.time()
    def __start_read_command(self, command:BICCommand):
        for _ in range(self.__num_repeats):
            data = [ command.value & 0xFF, (command.value & 0xFF00) >> 8]
            self.__canbus.send(threadsafe_can.Message(arbitration_id=self.__command_arb_id, dlc=2, data=data, is_extended_id=True))

            now = time.time()
            if now - self.__last_command_time < self.__command_interval_s: 
                time.sleep(now - self.__last_command_time)

            self.__last_command_time = time.time()

        # Reply will be received asynchronously

    def configure_bic(self):
        """Configure the BIC for bidirectional mode. Only needs to happen once (stored in flash). 
        Reboot after executing this command"""

        self.__write_command(BICCommand.SYSTEM_CONFIG, 0x0003, 2)
        self.__write_command(BICCommand.BIDIRECTIONAL_CONFIG, 0x001, 2)
        logger.warning("Commands set, please power off and on the BIC")