import extra.PYLON_EMU.pylon_cyc as pylon_sim
import threading

from threadsafe_can import Bus

def start(config):
    def run_battery_simulator():
        pylon_sim.test_periodic_send_with_modifying_data(
            Bus(**config["battery"]["canbus"]), 
            'extra/PYLON_EMU/pylon_CAN_210124.dbc')
    threading.Thread(target=run_battery_simulator, daemon=True).start()