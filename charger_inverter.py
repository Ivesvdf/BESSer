import threading
import time
import threadsafe_can
from loguru import logger
from enum import Enum


class BICCommand(Enum):
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


class BICFlags(Enum):
    def parse(value):
        """Receives an integer representing the device status flags and returns a dictionary containing the status of each flag"""
        flags = set()
        for flag in BICFlags:
            if value & flag.value:
                flags.add(flag)
        return flags


class FaultStatusFlag(BICFlags):
    # Fan locked flag (0 = Fan working normally, 1 = Fan locked)
    FAN_LOCKED = 1 << 0
    # Over temperature protection (0 = Internal temperature normal, 1 = Internal temperature abnormal)
    OVER_TEMP = 1 << 1
    # DC over voltage protection (0 = DC voltage normal, 1 = DC over voltage protected)
    DC_OVER_VOLTAGE = 1 << 2
    # DC overcurrent protection (0 = DC current normal, 1 = DC over current protected)
    DC_OVER_CURRENT = 1 << 3
    # Short circuit protection (0 = Shorted circuit does not exist, 1 = Shorted circuit protected)
    SHORT_CIRCUIT = 1 << 4
    # AC abnormal flag (0 = AC range normal, 1 = AC range abnormal)
    AC_ABNORMAL = 1 << 5
    # DC status (0 = DC turned on, 1 = DC turned off)
    DC_TURNED_OFF = 1 << 6
    # Internal high temperature protection (0 = Internal temperature normal, 1 = Internal temperature abnormal)
    INTERNAL_TEMP_ABNORMAL = 1 << 7
    # HV over voltage protection (0 = HV voltage normal, 1 = HV over voltage protected)
    HV_OVER_VOLTAGE = 1 << 8


class SystemStatusFlags(BICFlags):
    # Parallel mode status (0 = Current device is Slave, 1 = Current device is Master)
    PARALLEL_MODE_STATUS = 1 << 0
    # Secondary DD output voltage status (0 = Secondary DD output voltage status TOO LOW, 1 = Secondary DD output voltage status NORMAL)
    SECONDARY_DD_OUTPUT_VOLTAGE_STATUS = 1 << 1
    # Primary PFC status (0 = Primary PFC OFF or abnormal, 1 = Primary PFC ON normally)
    PRIMARY_PFC_STATUS = 1 << 2
    # Active dummy load control status (0 = Active dummy load off/function not supported, 1 = Active dummy load on)
    ACTIVE_DUMMY_LOAD_CONTROL_STATUS = 1 << 4
    # Device initialized status (0 = In initialization status, 1 = NOT in initialization status)
    DEVICE_INITIALIZED_STATUS = 1 << 5
    # EEPROM data access error (0 = EEPROM data access normal, 1 = EEPROM data access error)
    EEPROM_DATA_ACCESS_ERROR = 1 << 6


class SystemConfig:
    """
    Represents the CANBus communication control status and the pre-set value of
    power on operation command as a class.
    """
    class CANCtrl(Enum):
        """
        Represents the bit flags for the CANBus communication control status.
        """
        DISABLED = 0
        ENABLED = 1

        def __str__(self):
            return self.name

    class OperationInit(Enum):
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

class BICModel(Enum):
    BIC2200_12 = 12
    BIC2200_24 = 24
    BIC2200_48 = 48
    BIC2200_96 = 96

# Define the VOUT_SET portion of the table as a list of dictionaries, one per row
vout_table = {
    BICModel.BIC2200_12: {"adjustable range": (10, 15), "tolerance": 0.12, "default": 12},
    BICModel.BIC2200_24: {"adjustable range": (19, 28), "tolerance": 0.24, "default": 24},
    BICModel.BIC2200_48: {"adjustable range": (38, 65), "tolerance": 0.48, "default": 48},
    BICModel.BIC2200_96: {"adjustable range": (76, 112), "tolerance": 0.96, "default": 96}
}


# Define the REVERSE_VOUT_SET portion of the table as a list of dictionaries, one per row
reverse_vout_table = {
    BICModel.BIC2200_12: {"adjustable range": (-15, -10), "tolerance": 0.12, "default": -12},
    BICModel.BIC2200_24: {"adjustable range": (-28, -19), "tolerance": 0.24, "default": -24},
    BICModel.BIC2200_48: {"adjustable range": (-65, -38), "tolerance": 0.48, "default": -48},
    BICModel.BIC2200_96: {"adjustable range": (-112, -76), "tolerance": 0.96, "default": -96}
}

# Define the VOUT_SET portion of the table as a list of dictionaries, one per row
iout_table = {
    BICModel.BIC2200_12: {"adjustable range": (36, 198), "tolerance": 4, "default": 198},
    BICModel.BIC2200_24: {"adjustable range": (18, 99), "tolerance": 2, "default": 99},
    BICModel.BIC2200_48: {"adjustable range": (9, 49.5), "tolerance": 1, "default": 49.5},
    BICModel.BIC2200_96: {"adjustable range": (4.5, 24.75), "tolerance": 0.5, "default": 24.75}
}


# Define the REVERSE_VOUT_SET portion of the table as a list of dictionaries, one per row
reverse_iout_table = {
    BICModel.BIC2200_12: {"adjustable range": (-153, -36), "tolerance": 4, "default": -153},
    BICModel.BIC2200_24: {"adjustable range": (-76.5, -18), "tolerance": 2, "default": -76.5},
    BICModel.BIC2200_48: {"adjustable range": (-38.3, -9), "tolerance": 1, "default": -38.3},
    BICModel.BIC2200_96: {"adjustable range": (-19.1, -4.5), "tolerance": 0.5, "default": -19.1}
}


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
    def __init__(self, can: threadsafe_can.ThreadSafeCanInterface, device_id: int, model: BICModel,
                 battery_voltage_limits_V, Ki: float, disconnect_invert_V: float):
        self.__canbus = can
        self.__canbus.add_receive_hook(self.__on_can_receive)

        if device_id > 7 or device_id < 0:
            logger.error("Invalid device id, device id should range from 0 to 7")
            device_id = 0

        self.__response_arb_id = 0x000C0200 | device_id
        self.__command_arb_id = 0x000C0300 | device_id
        self.__receive_broadcast_arb_id = 0x000C03FF

        self.__model = model
        self.__battery_voltage_limits = battery_voltage_limits_V
        self.__Ki = Ki
        self.__power_control_current_mode = True
        self.__disconnect_invert_V = disconnect_invert_V

        self.__last_command_time = 0
        self.__command_interval_s = 0.025
        self.__num_repeats = 3
        self.__read_state = {}
        self.__write_state = {}

        self.__operational = False
        self.__can_control = None

        self.__Vin_factor = None
        self.__Vout_factor = None
        self.__Iout_factor = None
        self.__temperature_factor = None

        self.__Vin_V = None
        self.__Vout_V = None
        self.__Iout_A = None
        self.__temperature_C = None

        self.__fault_flags = set()
        self.__system_status = set()

        self.__cycle_time_basic_s = 0

        self.__target_power_W = 0
        self.__target_voltage_V = 0
        self.__target_current_A = 0

        self.__communication_thread = threading.Thread(target=self.__run, daemon=True)
        self.__receive_stop_event = threading.Event()
        self.__communication_thread.start()

    @property
    def Charge_Power_Limit_W(self) -> int:
        return -1725

    @property
    def Discharge_Power_Limit_W(self) -> int:
        return 2160

    @property
    def Charge_Iout_Limits_A(self):
        return iout_table[self.__model]["adjustable range"]

    @property
    def Discharge_Iout_Limits_A(self):
        return reverse_iout_table[self.__model]["adjustable range"]

    @property
    def Battery_Voltage_Limits_V(self):
        return self.__battery_voltage_limits_V

    @Battery_Voltage_Limits_V.setter
    def Battery_Voltage_Limits_V(self, value):
        self.__battery_voltage_limits_V = value

    @property
    def Vin_V(self) -> float:
        return self.__Vin_V

    @property
    def Vout_V(self) -> float:
        return self.__Vout_V

    @property
    def Iout_A(self) -> float:
        return self.__Iout_A

    @property
    def Current_Power_W(self) -> int:
        return (self.__Vout_V * self.__Iout_A) if (self.__Vout_V != None) and (self.__Iout_A != None) else None

    @property
    def Temperature_C(self) -> float:
        return self.__temperature_C

    @property
    def Fault_Flags(self) -> set():
        return set(self.__fault_flags)

    @property
    def System_Status(self) -> set():
        return set(self.__system_status)

    @property
    def Operational(self) -> bool:
        return set(self.__operational)

    @property
    def Target_Voltage_V(self) -> float:
        return self.__target_voltage_V

    @property
    def Target_Current_A(self) -> float:
        return self.__target_current_A

    @property
    def Target_Power_W(self) -> int:
        return self.__target_power_W

    @Target_Power_W.setter
    def Target_Power_W(self, value: int):
        if value < self.Charge_Power_Limit_W:
            self.__target_power_W = self.Charge_Power_Limit_W
        elif value > self.Discharge_Power_Limit_W:
            self.__target_power_W = self.Discharge_Power_Limit_W
        elif -100 < value < 100:
            self.__target_power_W = 0
        else:
            self.__target_power_W = value

    @property
    def Ki(self) -> float:
        return self.__Ki

    @Ki.setter
    def Ki(self, value: float):
        if value <= 0:
            raise ValueError("Ki must be greater than 0")

        self.__Ki = value

    @property
    def Last_cycle_time_basic_s(self):
        return self.__cycle_time_basic_s

    def __calculate_setpoint(self):
        (min_batt_voltage_V, max_batt_voltage_V) = self.__battery_voltage_limits
        (min_charge_current_limit_A, max_charge_current_limit_A) = iout_table[BICModel.BIC2200_48]["adjustable range"]
        (min_discharge_current_limit_A, max_discharge_current_limit_A) = reverse_iout_table[BICModel.BIC2200_48]["adjustable range"]

        self.__target_current_A = self.__target_power_W / self.__Vout_V

        if max_discharge_current_limit_A < self.__target_current_A < min_charge_current_limit_A:
            # outside of constant current range, use voltage control
            if self.__power_control_current_mode:
                self.__power_control_current_mode = False
                self.__target_voltage_V = self.__Vout_V

            if self.__target_current_A < 0:
                self.__target_current_A = max_discharge_current_limit_A
            else:
                self.__target_current_A = min_charge_current_limit_A

            self.__target_voltage_V += (self.__Ki * (self.__target_power_W - self.Current_Power_W))

            if self.__target_voltage_V < min_batt_voltage_V:
                self.__target_voltage_V = min_batt_voltage_V
            elif self.__target_voltage_V > max_batt_voltage_V:
                self.__target_voltage_V = max_batt_voltage_V
        else:
            # within the current control range, use constant current
            self.__power_control_current_mode = True

            if self.__target_current_A > 0:
                # charge
                if self.__target_current_A > max_charge_current_limit_A:
                    self.__target_current_A = max_charge_current_limit_A

                self.__target_voltage_V = max_batt_voltage_V
            else:
                # discharge
                if self.__target_current_A < min_discharge_current_limit_A:
                    self.__target_current_A = min_discharge_current_limit_A

                self.__target_voltage_V = min_batt_voltage_V

        logger.info(f"Will request charger/inverter for charge current {self.__target_current_A} \
                        with target voltage {self.__target_voltage_V}")

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
                logger.error(
                    "Your BIC had to be reprogrammed for CAN communication. \
                        Please power off and on your inverter/charger and restart this software.")
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

            if self.__Vout_V == None or self.__Iout_A == None or self.__Vin_V == None or self.__temperature_C == None:
                logger.info("Still starting, will not execute")
            else:
                operation_requested = self.__target_power_W != 0

                # Do not invert when the AC net voltage is too high
                if (self.__target_power_W < 0) and (self.__Vin_V > self.__disconnect_invert_V):
                    self.__fault_flags.add(FaultStatusFlag.AC_ABNORMAL)

                if len(self.__fault_flags) > 0:
                    operation_requested = 0

                if operation_requested != self.__operational:
                    self.__write_command(BICCommand.OPERATION, 1 if operation_requested else 0, 1)

                if operation_requested:
                    self.__calculate_setpoint()

                    if self.__target_current_A > 0:
                        # charge
                        self.__write_command(BICCommand.DIRECTION_CTRL, 0, 1)
                        self.__write_command(BICCommand.VOUT_SET, (int)(self.__target_voltage_V / self.__Vout_factor), 2)
                        self.__write_command(BICCommand.IOUT_SET, (int)(self.__target_current_A / self.__Iout_factor), 2)
                    else:
                        # discharge
                        self.__write_command(BICCommand.DIRECTION_CTRL, 1, 1)
                        self.__write_command(BICCommand.REVERSE_VOUT_SET, (int)(self.__target_voltage_V / self.__Vout_factor), 2)
                        self.__write_command(BICCommand.REVERSE_IOUT_SET, (int)(abs(self.__target_current_A) / self.__Iout_factor), 2)
                else:
                    self.__power_control_current_mode = True

                    self.__write_command(BICCommand.IOUT_SET, 0, 2)
                    self.__write_command(BICCommand.REVERSE_IOUT_SET, 0, 2)

            self.__cycle_time_basic_s = time.time() - cycle_time_start

            time.sleep(0.1)

    def get_write_state(self):
        return dict(self.__write_state)

    def get_read_state(self):
        return dict(self.__read_state)

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

    def __on_response_received(self, command: BICCommand, data: int):
        self.__read_state[command] = data
        if command == BICCommand.FAULT_STATUS:
            self.__fault_flags = FaultStatusFlag.parse(data)
        elif command == BICCommand.SYSTEM_STATUS:
            self.__system_status = SystemStatusFlags.parse(data)
        elif command == BICCommand.SYSTEM_CONFIG:
            self.__can_control = (data & 1) != 0
        elif command == BICCommand.SCALING_FACTOR:
            self.__Vout_factor = parse_factor(data & 0x0F)
            self.__Iout_factor = parse_factor((data & 0xF0) >> 4)
            self.__Vin_factor = parse_factor((data & 0x0F00) >> 8)
            self.__temperature_factor = parse_factor((data & 0x0F0000) >> 16)
        elif command == BICCommand.OPERATION:
            self.__operational = (data == 1)
        elif command == BICCommand.READ_VOUT:
            self.__Vout_V = data * self.__Vout_factor
        elif command == BICCommand.READ_TEMPERATURE_1:
            self.__temperature_C = from_twos_complement(data, 16) * self.__temperature_factor
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
                    command = BICCommand(data[0] | (data[1] << 8))
                    data = data[2] | (data[3] << 8) | (data[4] << 16) | (data[5] << 24)
                except ValueError:
                    logger.error(f"Invalid response received: {data}")

                self.__on_response_received(command, data)
        except:
            logger.exception("Unknown BIC response received")

    def __write_command(self, command: BICCommand, value: int, num_data_bytes: int):
        self.__write_state[command] = value

        for _ in range(self.__num_repeats):
            data = [command.value & 0xFF, (command.value & 0xFF00) >> 8, (value & 0xFF), (value & 0xFF00) >> 8]
            self.__canbus.send(threadsafe_can.Message(arbitration_id=self.__command_arb_id,
                               dlc=2+num_data_bytes, data=data, is_extended_id=True))

            now = time.time()
            if now - self.__last_command_time < self.__command_interval_s:
                time.sleep(now - self.__last_command_time)

        self.__last_command_time = time.time()

    def __start_read_command(self, command: BICCommand):
        for _ in range(self.__num_repeats):
            data = [command.value & 0xFF, (command.value & 0xFF00) >> 8]
            self.__canbus.send(threadsafe_can.Message(
                arbitration_id=self.__command_arb_id, dlc=2, data=data, is_extended_id=True))

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
