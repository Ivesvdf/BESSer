import time
import threadsafe_can
from loguru import logger
from enum import IntEnum
import threading

class ProtectionFlags(IntEnum):
    DISCHARGE_OVERCURRENT = 0x80  # byte 0, bit 7
    CELL_UNDERTEMP = 0x10  # byte 0, bit 4
    CELL_OVERTEMP = 0x08  # byte 0, bit 3
    CELL_MODULE_UNDERVOLT = 0x04  # byte 0, bit 2
    CELL_MODULE_OVERVOLT = 0x02  # byte 0, bit 1
    SYSTEM_ERROR = 0x08  # byte 1, bit 3
    CHARGE_OVERCURRENT = 0x01  # byte 1, bit 0

class AlarmFlags(IntEnum):
    DISCHARGE_HIGHCURRENT = 0x80  # byte 2, bit 7
    CELL_LOWTEMP = 0x10  # byte 2, bit 4
    CELL_HIGHTEMP = 0x08  # byte 2, bit 3
    CELL_MODULE_LOWVOLT = 0x04  # byte 2, bit 2
    CELL_MODULE_HIGHVOLT = 0x02  # byte 2, bit 1
    INTERNAL_COMMS_FAIL = 0x08  # byte 3, bit 3
    CHARGE_HIGHCURRENT = 0x01  # byte 3, bit 0

class RequestFlags(IntEnum):
    CHARGE_ENABLE = 0x80  # byte 0, bit 7
    DISCHARGE_ENABLE = 0x40  # byte 0, bit 6
    REQUEST_FORCE_CHARGE_1 = 0x20  # byte 0, bit 5
    REQUEST_FORCE_CHARGE_2 = 0x10  # byte 0, bit 4
    REQUEST_FULL_CHARGE = 0x08  # byte 0, bit 3

class RepeatTimer(threading.Timer):
    def run(self):
        while not self.finished.wait(self.interval):
            self.function(*self.args, **self.kwargs)

class PylontechCANBattery:
    def __init__(self, canbus: threadsafe_can.ThreadSafeCanInterface):
        self.__batt_V = None
        self.__batt_A = None
        self.__batt_T = None

        self.__batt_charge_V = None
        self.__batt_discharge_V = None
        self.__batt_charge_A = None
        self.__batt_discharge_A = None

        self.__protection_flags = None
        self.__alarm_flags = None
        self.__request_flags = None

        self.__batt_soc_pct = None
        self.__batt_soh_pct = None

        self._alive_packet_counter = 0

        self._core_ids = [ 0x351, 0x355, 0x359, 0x35c, 0x35e ]
        self._last_receive_time = { x: 0 for x in self._core_ids }

        self.__canbus = canbus
        self.__canbus.add_receive_hook(self.process_can)

        RepeatTimer(interval=1, function=self.send_heartbeat).start()


    def get_batt_V(self):
        return self.__batt_V

    def get_batt_A(self):
        return self.__batt_A

    def get_batt_T(self):
        return self.__batt_T

    def get_batt_charge_V(self):
        return self.__batt_charge_V
    
    def get_batt_discharge_V(self):
        return self.__batt_discharge_V
    
    def get_batt_charge_A(self):
        return self.__batt_charge_A

    def get_batt_discharge_A(self):
        return self.__batt_discharge_A

    def get_protection_flags(self):
        return self.__protection_flags

    def get_alarm_flags(self):
        return self.__alarm_flags

    def get_request_flags(self):
        return self.__request_flags

    def get_batt_soc_pct(self):
        return self.__batt_soc_pct

    def get_batt_soh_pct(self):
        return self.__batt_soh_pct
    
    def get_protection_flags(self):
        return set(self.__protection_flags) if self.__protection_flags != None else None

    def get_alarm_flags(self):
        return set(self.__alarm_flags)  if self.__alarm_flags != None else None

    def get_request_flags(self):
        return set(self.__request_flags)  if self.__request_flags != None else None

    def send_heartbeat(self):
        message = threadsafe_can.Message(arbitration_id=0x305, dlc=8, is_extended_id=False)
        message.data = [ 0 for _ in range(8)]
        message.data[0] = self._alive_packet_counter & 0xFF;
        message.data[1] = (self._alive_packet_counter >>  8) & 0xFF; 
        message.data[2] = (self._alive_packet_counter >>  16) & 0xFF; 
        message.data[3] = (self._alive_packet_counter >>  24) & 0xFF;
        message.data[4] = 0x00;
        message.data[5] = 0x00; 
        message.data[6] = 0x00; 
        message.data[7] = 0x00; 

        self.__canbus.send(message)

        self._alive_packet_counter += 1

    def get_oldest_receive_time(self):
        return min([v for k,v in self._last_receive_time.items() if k in self._core_ids ])
             
    def process_can(self, message):
        can_id = message.arbitration_id
        can_buffer = message.data

        #logger.debug(f"Received {message}")

        self._last_receive_time[message.arbitration_id] = time.time()

        # 0x305 - Inverter reply (we ignore)
        # 0x351 - Battery voltage + current limits
        # 0x355 - State of Health (SOH) / State of Charge (SOC)
        # 0x356 - Voltage / Current / Temp
        # 0x359 - Protection & Alarms
        # 0x35C - Battery charge request
        # 0x35E - Manufacturer name (PYLON)

        # loop through the buffer, newest record working backwards..
        if can_id == 0x351:
            # Battery charge voltage, charge/discharge current limit - 6 bytes of data
            # 16 bits, unsigned int, signed int, signed int
            # V in 0.1, A in 0.1
            val_unsigned = (can_buffer[1] << 8) + can_buffer[0]
            self.__batt_charge_V = val_unsigned / 10

            val_signed = (can_buffer[3] << 8) + can_buffer[2]
            if val_signed > 0x7FFF:
                val_signed -= 0x10000
            self.__batt_charge_A = val_signed / 10

            val_signed = (can_buffer[5] << 8) + can_buffer[4]
            if val_signed > 0x7FFF:
                val_signed -= 0x10000
            self.__batt_discharge_A = val_signed / 10

            val_signed = (can_buffer[7] << 8) + can_buffer[6]
            if val_signed > 0x7FFF:
                val_signed -= 0x10000
            self.__batt_discharge_V = val_signed / 10

            logger.info(f"Charge Voltage={self.__batt_charge_V}V, Charge Current={self.__batt_charge_A}A, Discharge Current={self.__batt_discharge_A}A")

        elif can_id == 0x355:
            # State of Charge (SOC) / State of Health (SOH) - 4 bytes of data
            # 16 bits, unsigned int
            # value as a percent
            self.__batt_soc_pct = (can_buffer[1] << 8) + can_buffer[0]
            self.__batt_soh_pct = (can_buffer[3] << 8) + can_buffer[2]

            logger.info(f"SOC={self.__batt_soc_pct}%, SOH={self.__batt_soh_pct}%")

        elif can_id == 0x356:
            # Voltage / Current / Temp - 6 bytes of data
            # 16 bits, signed int, 2s complement
            # V in 0.01V, A in 0.1A, T in 0.1C
            val_signed = (can_buffer[1] << 8) + can_buffer[0]
            if val_signed > 0x7FFF:
                val_signed -= 0x10000
            self.__batt_V = val_signed / 100

            val_signed = (can_buffer[3] << 8) + can_buffer[2]
            if val_signed > 0x7FFF:
                val_signed -= 0x10000
            self.__batt_A = val_signed / 10

            val_signed = (can_buffer[5] << 8) + can_buffer[4]
            if val_signed > 0x7FFF:
                val_signed -= 0x10000
            self.__batt_T = val_signed / 10

            logger.info(f"ID: 0x356 Volts={self.__batt_V}, Current={self.__batt_A}, Temp={self.__batt_T}")

        elif can_id == 0x359:
            protection_flags = set()
            alarm_flags = set()
            byte_0 = can_buffer[0]
            byte_1 = can_buffer[1]
            byte_2 = can_buffer[2]
            byte_3 = can_buffer[3]

            if byte_0 & ProtectionFlags.DISCHARGE_OVERCURRENT:
                protection_flags.add(ProtectionFlags.DISCHARGE_OVERCURRENT)
            if byte_0 & ProtectionFlags.CELL_UNDERTEMP:
                protection_flags.add(ProtectionFlags.CELL_UNDERTEMP)
            if byte_0 & ProtectionFlags.CELL_OVERTEMP:
                protection_flags.add(ProtectionFlags.CELL_OVERTEMP)
            if byte_0 & ProtectionFlags.CELL_MODULE_UNDERVOLT:
                protection_flags.add(ProtectionFlags.CELL_MODULE_UNDERVOLT)
            if byte_0 & ProtectionFlags.CELL_MODULE_OVERVOLT:
                protection_flags.add(ProtectionFlags.CELL_MODULE_OVERVOLT)
            if byte_1 & ProtectionFlags.SYSTEM_ERROR:
                protection_flags.add(ProtectionFlags.SYSTEM_ERROR)
            if byte_1 & ProtectionFlags.CHARGE_OVERCURRENT:
                protection_flags.add(ProtectionFlags.CHARGE_OVERCURRENT)
            if byte_2 & AlarmFlags.DISCHARGE_HIGHCURRENT:
                alarm_flags.add(AlarmFlags.DISCHARGE_HIGHCURRENT)
            if byte_2 & AlarmFlags.CELL_LOWTEMP:
                alarm_flags.add(AlarmFlags.CELL_LOWTEMP)
            if byte_2 & AlarmFlags.CELL_HIGHTEMP:
                alarm_flags.add(AlarmFlags.CELL_HIGHTEMP)
            if byte_2 & AlarmFlags.CELL_MODULE_LOWVOLT:
                alarm_flags.add(AlarmFlags.CELL_MODULE_LOWVOLT)
            if byte_2 & AlarmFlags.CELL_MODULE_HIGHVOLT:
                alarm_flags.add(AlarmFlags.CELL_MODULE_HIGHVOLT)
            if byte_3 & AlarmFlags.INTERNAL_COMMS_FAIL:
                alarm_flags.add(AlarmFlags.INTERNAL_COMMS_FAIL)
            if byte_3 & AlarmFlags.CHARGE_HIGHCURRENT:
                alarm_flags.add(AlarmFlags.CHARGE_HIGHCURRENT)

            self.__protection_flags = protection_flags
            self.__alarm_flags = alarm_flags

            logger.info(f"ID: 0x359 Protection / Alarm Flags: {protection_flags}, {alarm_flags}")

        if can_id == 0x35c:
            # Request flags (0x35C)
            # byte 0: Request flags
            flags = set()

            byte_0 = can_buffer[0]
            if byte_0 & RequestFlags.CHARGE_ENABLE:
                flags.add(RequestFlags.CHARGE_ENABLE)
            if byte_0 & RequestFlags.DISCHARGE_ENABLE:
                flags.add(RequestFlags.DISCHARGE_ENABLE)
            if byte_0 & RequestFlags.REQUEST_FORCE_CHARGE_1:
                flags.add(RequestFlags.REQUEST_FORCE_CHARGE_1)
            if byte_0 & RequestFlags.REQUEST_FORCE_CHARGE_2:
                flags.add(RequestFlags.REQUEST_FORCE_CHARGE_2)
            if byte_0 & RequestFlags.REQUEST_FULL_CHARGE:
                flags.add(RequestFlags.REQUEST_FULL_CHARGE)
            self.__request_flags = flags
            logger.info(F"ID: 0x35C Request Flags: {flags}")

        elif can_id == 0x35E:
            # Manufacturer Name: "PYLON  "
            logger.info(F"ID: 0x35E {[ chr(can_buffer[i]) for i in range(len(can_buffer)) ]}")