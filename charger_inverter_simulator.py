from charger_inverter import BICCommand, to_twos_complement, from_twos_complement
import threadsafe_can as can 

class BICSimulator:
    def __init__(self, bus):
        self.__bus = bus
        self.__write_arb_id = 0x000C0300
        self.__read_arb_id = 0x000C0200
        self.__responses = {}

        self.__internal_voltage_V = 51.0
        self.__set_V = 51.0
        self.__reverse_set_V = 51.0
        self.__operation = 0
        self.__direction = 0
        self.__current_A = 0
        self.__internal_resistance_ohm = 0.05
        self.__current_limit_A = 9
        self.__reverse_current_limit_A = 9

        self.ratio = 0.01
        self.add_response(BICCommand.READ_VOUT, 51/self.ratio)
        self.add_response(BICCommand.READ_VIN, 230/self.ratio)
        self.add_response(BICCommand.READ_TEMPERATURE_1, to_twos_complement(int(-23/self.ratio), 16))
        self.add_response(BICCommand.READ_IOUT, to_twos_complement(int(-0/self.ratio), 16))
        self.add_response(BICCommand.SCALING_FACTOR, 0x55555555)
        self.add_response(BICCommand.SYSTEM_CONFIG, 3)

    def __write_command(self, command:BICCommand, value:int): 
        data = [ command.value & 0xFF, (command.value & 0xFF00) >> 8, (value & 0xFF), (value & 0xFF00) >> 8 ]
        self.__bus.send(can.Message(arbitration_id=self.__write_arb_id, data=data, is_extended_id=True))

    def __start_read_command(self, command:BICCommand):
        data = [ command.value & 0xFF, (command.value & 0xFF00) >> 8 ]
        self.__bus.send(can.Message(arbitration_id=self.__read_arb_id, data=data, is_extended_id=True))

    def add_response(self, command:BICCommand, value:int):
        value = int(value)
        self.__responses[command] = value

    def simulate(self):
        for msg in self.__bus:
            
            if self.__direction == 0:
                target_V = self.__set_V
            else:
                target_V = self.__reverse_set_V

            cur_A = (target_V - self.__internal_voltage_V)/self.__internal_resistance_ohm

            if self.__direction == 0:
                if cur_A > self.__current_limit_A:
                    cur_A = self.__current_limit_A
                if cur_A < 0:
                    cur_A = -1
            else:
                if cur_A < self.__reverse_current_limit_A:
                    cur_A = self.__reverse_current_limit_A
                if cur_A > 0:
                    cur_A = 1
                    
            if self.__operation == 0:
                cur_A = 0
    
            out_V = self.__internal_voltage_V + cur_A * self.__internal_resistance_ohm
            self.add_response(BICCommand.READ_VOUT, out_V/self.ratio)
            self.add_response(BICCommand.READ_IOUT, cur_A/self.ratio)

            twobytedata = (from_twos_complement(msg.data[2] | msg.data[3] << 8, 16) * self.ratio) if len(msg.data) > 3 else 0
            if msg.arbitration_id == self.__write_arb_id:
                command = BICCommand((msg.data[1] << 8) | msg.data[0])
                value = 0
                for i in range(2,8):
                    if i < len(msg.data):
                        value += msg.data[i] << (8*(len(msg.data)-i))
                response_value = self.__responses.get(command, 0)
                if command == BICCommand.VOUT_SET:
                    self.__set_V = twobytedata
                if command == BICCommand.REVERSE_VOUT_SET:
                    self.__reverse_set_V = twobytedata
                if command == BICCommand.OPERATION:
                    if len(msg.data) > 2:
                        self.__operation = msg.data[2]
                    else:
                        response_value = self.__operation
                if command == BICCommand.DIRECTION_CTRL:
                    if len(msg.data) > 2:
                        self.__direction = msg.data[2]
                if command == BICCommand.IOUT_SET:
                    self.__current_limit_A = twobytedata if twobytedata > 9 else 9
                if command == BICCommand.REVERSE_IOUT_SET:
                    self.__reverse_current_limit_A = twobytedata if twobytedata < -9 else -9
                if command == BICCommand.READ_IOUT:
                    response_value = to_twos_complement(response_value, 16)

                response_data = [msg.data[0], msg.data[1], 
                                    (response_value & 0xFF) >> 0, (response_value & 0xFF00) >> 8,
                                    (response_value & 0xFF0000) >> 16, (response_value & 0xFF000000) >> 24,
                                    (response_value & 0xFF00000000) >> 32, (response_value & 0xFF0000000000) >> 48,
                                    (response_value & 0xFF000000000000) >> 56,

                                    ]
                response_msg = can.Message(arbitration_id=self.__read_arb_id, data=response_data, is_extended_id=True)
                self.__bus.send(response_msg)