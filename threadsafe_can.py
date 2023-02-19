import can
import threading

from can import Bus, Message


class ThreadSafeCanInterface:
    def __init__(self, bus: can.Bus):
        self.__bus = bus
        self.__receive_hooks = []

        self.__lock = threading.Lock()

        self.__receive_thread = threading.Thread(target=self.receive_loop, daemon=True)
        self.__receive_stop_event = threading.Event()
        self.__receive_thread.start()

    def send(self, message: can.Message):
        with self.__lock:
            self.__bus.send(message)

    def add_receive_hook(self, hook):
        with self.__lock:
            self.__receive_hooks.append(hook)

    def remove_receive_hook(self, hook):
        with self.__lock:
            self.__receive_hooks.remove(hook)

    def stop_receive_thread(self):
        self.__receive_stop_event.set()
        self.__receive_thread.join()

    def receive_loop(self):
        while not self.__receive_stop_event.is_set():
            with self.__lock:
                message = self.__bus.recv()
            for hook in self.__receive_hooks:
                hook(message)
                
        self.__bus.shutdown()

    def __del__(self):
        self.stop_receive_thread()



if __name__ == "__main__":
    # Example usage
    def handle_message1(message):
        print(f"Received message (hook 1): {message}")

    def handle_message2(message):
        print(f"Received message (hook 2): {message}")

    bus = can.Bus(interface='socketcan', channel='can0')
    can_interface = ThreadSafeCanInterface(bus)
    can_interface.add_receive_hook(handle_message1)
    can_interface.add_receive_hook(handle_message2)
    can_interface.send(can.Message(arbitration_id=0x123, data=[1, 2, 3]))

    # ... program continues ...

    # Stop receive thread when program is done
    can_interface.stop_receive_thread()