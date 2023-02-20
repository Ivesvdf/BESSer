import enum
import time

import battery
import charger_inverter
import config_validator
import config
import mqtt_interface
import threadsafe_can
from battery import AlarmFlags, ProtectionFlags, RequestFlags

battery_can_interface = threadsafe_can.ThreadSafeCanInterface(config.battery_canbus)
inverter_can_interface = threadsafe_can.ThreadSafeCanInterface(config.charger_inverter_canbus)

mqtt = mqtt_interface.MqttInterface(config.mqtt_connect_args, config.mqtt_topic_prefix)

charger_inverter = charger_inverter.BICChargerInverter(inverter_can_interface, 
                                                       config.charger_inverter_device_id, 
                                                       config.charger_inverter_model_voltage, 
                                                       (config.battery_min_voltage, config.battery_max_voltage))
battery = battery.PylontechCANBattery(battery_can_interface)

mqtt_requested_power_W = None
battery_requested_power_W = None 

def on_mqtt_power_request(power_W):
    global mqtt_requested_power_W
    mqtt_requested_power_W = power_W 

def on_configure_bic_request():
    global charger_inverter
    charger_inverter.configure_bic()

last_heartbeat_timestamp = 0
def on_hearbeat():
    global last_heartbeat_timestamp
    last_heartbeat_timestamp = time.time()

mqtt.on_power_request = on_mqtt_power_request
mqtt.on_heartbeat = on_hearbeat

class DeviationReason(enum.Enum):
    CHARGE_ENABLE_NOT_SET = enum.auto()
    DISCHARGE_ENABLE_NOT_SET = enum.auto()
    REQUEST_FULL_CHARGE_SET = enum.auto()
    REQUEST_FORCE_CHARGE_SET = enum.auto()
    UNDERVOLT_FLAG_SET = enum.auto()
    OVERVOLT_FLAG_SET = enum.auto()
    PROTECTION_FLAG_SET = enum.auto()
    ALARM_FLAG_SET = enum.auto()
    NO_DATA_FROM_BATTERY = enum.auto()
    BATTERY_CURRENT_LIMIT = enum.auto()
    INVERTER_FAULT_SET = enum.auto()
    NO_MQTT_HEARTBEAT = enum.auto()
    CHARGE_ABOVE_MAX_SOC = enum.auto()
    DISCHARGE_BELOW_MIN_SOC = enum.auto()
    INVERTER_POWER_LIMIT = enum.auto()

last_broadcast_time = 0
last_broadcast_power_request_W = None

while True:
    protection_flags = battery.get_protection_flags() or set()
    alarm_flags = battery.get_alarm_flags() or set()
    request_flags = battery.get_request_flags() or set()

    deviation_reasons = set()
    
    soc_max = config.battery_max_soc_charge
    soc_min = config.battery_min_soc_discharge

    power_request_W = mqtt_requested_power_W or 0

    time_since_last_heartbeat_s = (config.mqtt_heartbeat_interval_s != 0 and time.time() - last_heartbeat_timestamp)
    if power_request_W != 0 and time_since_last_heartbeat_s > config.mqtt_heartbeat_interval_s:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.NO_MQTT_HEARTBEAT)

    batt_V = battery.get_batt_V()
    batt_A = battery.get_batt_A()
    batt_T = battery.get_batt_T()

    batt_charge_V = battery.get_batt_charge_V()
    batt_discharge_V = battery.get_batt_discharge_V()

    batt_charge_A = battery.get_batt_charge_A()
    batt_discharge_A = battery.get_batt_discharge_A()

    batt_soc_pct = battery.get_batt_soc_pct()
    batt_soh_pct = battery.get_batt_soh_pct()

    inverter_fault_flags = charger_inverter.get_fault_flags()

    if power_request_W > 0 and RequestFlags.CHARGE_ENABLE not in request_flags:
        power_request_W = max(0, power_request_W)
        deviation_reasons.add(DeviationReason.CHARGE_ENABLE_NOT_SET)
    if power_request_W < 0 and RequestFlags.DISCHARGE_ENABLE not in request_flags:
        power_request_W = min(0, power_request_W)
        deviation_reasons.add(DeviationReason.DISCHARGE_ENABLE_NOT_SET)

    if RequestFlags.REQUEST_FULL_CHARGE in request_flags:
        # Recharge fully by never discharging.
        # If we wanted to we could also charge from grid however
        # a few days to fully charge won't matter for calibration.
        soc_min = 99

        if power_request_W < 0:
            deviation_reasons.add(DeviationReason.REQUEST_FULL_CHARGE_SET)
    elif RequestFlags.REQUEST_FORCE_CHARGE_1 in request_flags or \
         RequestFlags.REQUEST_FORCE_CHARGE_2 in request_flags:
        soc_min = 20

        if power_request_W < 0:
            deviation_reasons.add(DeviationReason.REQUEST_FORCE_CHARGE_SET)

    # If any protection flag is set, only compensate for 
    if len(protection_flags) == 1:
        if ProtectionFlags.CELL_MODULE_UNDERVOLT in protection_flags:
            soc_min = 99
            deviation_reasons.add(DeviationReason.UNDERVOLT_FLAG_SET)
        elif ProtectionFlags.CELL_MODULE_OVERVOLT in protection_flags:
            soc_max = 10
            deviation_reasons.add(DeviationReason.OVERVOLT_FLAG_SET)
    elif len(protection_flags) > 1:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.PROTECTION_FLAG_SET)

    if len(alarm_flags) > 0:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.ALARM_FLAG_SET)

    if power_request_W > 0 and batt_soc_pct > soc_max:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.CHARGE_ABOVE_MAX_SOC)
    elif (power_request_W < 0 and batt_soc_pct < soc_min):
       power_request_W = 0
       deviation_reasons.add(DeviationReason.DISCHARGE_BELOW_MIN_SOC)

    oldest_receive_time = battery.get_oldest_receive_time()
    oldest_battery_receive_time_age = time.time() - oldest_receive_time

    batt_max_charge_W = (batt_V or 0) * (batt_charge_A or 0)
    batt_max_discharge_W = (batt_V or 0)*(batt_discharge_A or 0)


    if power_request_W > batt_max_charge_W:
        power_request_W = batt_max_charge_W
        deviation_reasons.add(DeviationReason.BATTERY_CURRENT_LIMIT)
    if power_request_W < batt_max_discharge_W:
        power_request_W = batt_max_discharge_W
        deviation_reasons.add(DeviationReason.BATTERY_CURRENT_LIMIT)

    if oldest_battery_receive_time_age > 30:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.NO_DATA_FROM_BATTERY)

    # From this point on we're talking about inverter limits. Even if the battery
    # really wants to be charged if the inverter can't do it we can't do it.
    if len(inverter_fault_flags) > 0: 
        power_request_W = 0
        deviation_reasons.add(DeviationReason.INVERTER_FAULT_SET)

    charge_power_limit_W = charger_inverter.get_charge_power_limit_W()
    invert_power_limit_W = charger_inverter.get_invert_power_limit_W()
    if power_request_W > charge_power_limit_W:
        power_request_W = charge_power_limit_W
        deviation_reasons.add(DeviationReason.INVERTER_POWER_LIMIT)
    elif power_request_W < invert_power_limit_W:
        power_request_W = invert_power_limit_W
        deviation_reasons.add(DeviationReason.INVERTER_POWER_LIMIT)


    charger_inverter.set_battery_voltage_limits((batt_discharge_V or config.battery_min_voltage, batt_charge_V or config.battery_max_voltage))
    charger_inverter.request_charge_discharge(power_request_W)

    status = battery_dict = {
        "batt_V": batt_V,
        "batt_A": batt_A,
        "batt_T": batt_T,
        "batt_charge_V": batt_charge_V,
        "batt_discharge_V": batt_discharge_V,
        "batt_charge_A": batt_charge_A,
        "batt_discharge_A": batt_discharge_A,
        "protection_flags": protection_flags,
        "alarm_flags": alarm_flags,
        "request_flags": request_flags,
        "batt_soc_pct": batt_soc_pct,
        "batt_soh_pct": batt_soh_pct,
        "deviation_reasons": deviation_reasons,
        "power_request": power_request_W,
        'inverter_DC_V': charger_inverter.get_Vout_V(),
        'inverter_temperature_1': charger_inverter.get_temperature_1(),
        'inverter_AC_V': charger_inverter.get_Vin_V(),
        'inverter_DC_A': charger_inverter.get_Iout_A(),
        'inverter_fault_flags': inverter_fault_flags,
        'inverter_system_status': charger_inverter.get_system_status(),
    }

    now = time.time()

    if (now - last_broadcast_time > config.mqtt_status_broadcast_interval_s) or (last_broadcast_power_request_W != power_request_W): 
        last_broadcast_time = now
        last_broadcast_power_request_W = power_request_W
        mqtt.broadcast_status(status)

    time.sleep(1)


# Stop the network loop
client.loop_stop()

# Disconnect from the broker
client.disconnect()