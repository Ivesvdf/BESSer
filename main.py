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

mqtt = mqtt_interface.MqttInterface(config.mqtt_connect_args, config.mqtt_credentials, config.mqtt_topic_prefix)

charger_inverter = charger_inverter.BICChargerInverter(inverter_can_interface, 
                                                       config.charger_inverter_device_id, 
                                                       config.charger_inverter_model_voltage, 
                                                       (config.battery_min_voltage, config.battery_max_voltage),
                                                       config.charger_inverter_PID_Ki, 
                                                       config.charger_inverter_PID_Kp, 
                                                       config.charger_inverter_PID_Kd,
                                                       config.charger_inverter_PID_Ki_min,
                                                       config.charger_inverter_PID_Ki_max)
battery = battery.PylontechCANBattery(battery_can_interface)

mqtt_requested_power_W = None
battery_requested_power_W = None 
min_soc_override = None
max_soc_override = None

charger_inverter_Ki = None
charger_inverter_Kp = None
charger_inverter_Kd = None 

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

def on_min_soc(soc):
    global min_soc_override
    min_soc_override = soc

def on_max_soc(soc):
    global max_soc_override
    max_soc_override = soc

def on_Ki(val):
    global charger_inverter_Ki
    charger_inverter_Ki = val

def on_Kp(val):
    global charger_inverter_Kp
    charger_inverter_Kp = val

def on_Kd(val):
    global charger_inverter_Kd
    charger_inverter_Kd = val

mqtt.on_power_request = on_mqtt_power_request
mqtt.on_heartbeat = on_hearbeat
mqtt.on_min_soc = on_min_soc
mqtt.on_max_soc = on_max_soc
mqtt.on_Ki = on_Ki
mqtt.on_Kp = on_Kp
mqtt.on_Kd = on_Kd

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
    GRID_OVER_VOLTAGE = enum.auto()
    REQUEST_BELOW_MIN_POWER = enum.auto()

last_broadcast_time = 0
last_broadcast_power_request_W = None
last_status = None
last_debug_info = None

while True:
    protection_flags = battery.get_protection_flags() or set()
    alarm_flags = battery.get_alarm_flags() or set()
    request_flags = battery.get_request_flags() or set()

    deviation_reasons = set()
    
    # Max soc can only be decreased through MQTT, min soc only increased. 
    soc_max = min(config.battery_max_soc_charge, max_soc_override or 100)
    soc_min = max(config.battery_min_soc_discharge, min_soc_override or 0)

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

    # Reset charge and discharge due to SoC limits
    if power_request_W > 0 and (batt_soc_pct or 0) > soc_max:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.CHARGE_ABOVE_MAX_SOC)
    elif (power_request_W < 0 and (batt_soc_pct or 0) < soc_min):
       power_request_W = 0
       deviation_reasons.add(DeviationReason.DISCHARGE_BELOW_MIN_SOC)

    oldest_receive_time = battery.get_oldest_receive_time()
    oldest_battery_receive_time_age = time.time() - oldest_receive_time

    batt_max_charge_W = (batt_V or 0) * (batt_charge_A or 0)
    batt_max_discharge_W = (batt_V or 0)*(batt_discharge_A or 0)

    # LLimit charge and discharge due to battery limits
    if power_request_W > batt_max_charge_W:
        power_request_W = batt_max_charge_W
        deviation_reasons.add(DeviationReason.BATTERY_CURRENT_LIMIT)
    if power_request_W < batt_max_discharge_W:
        power_request_W = batt_max_discharge_W
        deviation_reasons.add(DeviationReason.BATTERY_CURRENT_LIMIT)

    # If the mqtt heartbeat interval is set, do not charge or discharge if it hasn't been received
    if config.mqtt_heartbeat_interval_s != 0 and oldest_battery_receive_time_age > config.mqtt_heartbeat_interval_s:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.NO_DATA_FROM_BATTERY)

    # From this point on we're talking about inverter limits. Even if the battery
    # really wants to be charged if the inverter can't do it we can't do it.
    if len(inverter_fault_flags) > 0: 
        power_request_W = 0
        deviation_reasons.add(DeviationReason.INVERTER_FAULT_SET)

    # Limit charge and discharge power due to inverter limits
    charge_power_limit_W = charger_inverter.get_charge_power_limit_W()
    invert_power_limit_W = charger_inverter.get_invert_power_limit_W()
    if power_request_W > charge_power_limit_W:
        power_request_W = charge_power_limit_W
        deviation_reasons.add(DeviationReason.INVERTER_POWER_LIMIT)
    elif power_request_W < invert_power_limit_W:
        power_request_W = invert_power_limit_W
        deviation_reasons.add(DeviationReason.INVERTER_POWER_LIMIT)

    inverter_dc_V = charger_inverter.get_Vout_V()
    inverter_dc_A = charger_inverter.get_Iout_A()
    inverter_ac_V = charger_inverter.get_Vin_V()
    inverter_temp_degC = charger_inverter.get_temperature_1()

    if power_request_W != 0 and -config.min_invert_power_W < power_request_W < config.min_charge_power_W:
        power_request_W = 0
        deviation_reasons.add(DeviationReason.REQUEST_BELOW_MIN_POWER)

    # Do not invert when the AC net voltage is too high
    if power_request_W < 0 and inverter_ac_V > config.charger_inverter_disconnect_invert_V: 
        power_request_W = 0
        deviation_reasons.add(DeviationReason.GRID_OVER_VOLTAGE)

    if charger_inverter_Ki != None:
        charger_inverter.set_PID_Ki(charger_inverter_Ki)
        charger_inverter_Ki = None 

    if charger_inverter_Kp != None:
        charger_inverter.set_PID_Kp(charger_inverter_Kp)
        charger_inverter_Kp = None 

    if charger_inverter_Kd != None:
        charger_inverter.set_PID_Kd(charger_inverter_Kd)
        charger_inverter_Kd = None

    charger_inverter.set_battery_voltage_limits((batt_discharge_V or config.battery_min_voltage, batt_charge_V or config.battery_max_voltage))
    charger_inverter.request_charge_discharge(power_request_W)


    status = {
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
        "min_soc_override": min_soc_override,
        "max_soc_override": max_soc_override,
        "deviation_reasons": deviation_reasons,
        "power_request": power_request_W,
        'inverter_DC_V': round(inverter_dc_V, 3) if inverter_dc_V != None else None,
        'inverter_temperature_1': round(inverter_temp_degC, 3) if inverter_temp_degC != None else None,
        'inverter_AC_V': round(inverter_ac_V, 3) if inverter_ac_V != None else None,
        'inverter_DC_A': round(inverter_dc_A, 3) if inverter_dc_V != None else None,
        'inverter_DC_VA': round(inverter_dc_V * inverter_dc_A, 3) if inverter_dc_V != None and inverter_dc_A != None else 0,
        'inverter_fault_flags': inverter_fault_flags,
        'inverter_system_status': charger_inverter.get_system_status(),
    }


    def fix_dict(d):
        return { str(k): v for k,v in d.items() } 
    debug_info = { "inverter" : { 
                "out" : fix_dict(charger_inverter.get_write_state()),
                "in": fix_dict(charger_inverter.get_read_state()),
                "last_cycle_time": charger_inverter.get_last_cycle_time_basic_s(),
                "pid": charger_inverter.get_pid_state(),
             }}


    now = time.time()

    if (now - last_broadcast_time > config.mqtt_status_broadcast_interval_s) or (last_status != status) or (last_debug_info != debug_info): 
        last_broadcast_time = now
        last_status = status
        last_debug_info = debug_info
        mqtt.broadcast_status(status)
        mqtt.broadcast_debug(debug_info)


    time.sleep(1)


# Stop the network loop
client.loop_stop()

# Disconnect from the broker
client.disconnect()