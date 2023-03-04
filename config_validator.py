import config
import threadsafe_can as can

# Validate can buses
#assert isinstance(config.battery_canbus, can.Bus), "config.battery_canbus is not of type can.Bus"
#assert isinstance(config.charger_inverter_canbus, can.Bus), "config.charger_inverter_canbus is not of type can.Bus"

# Validate MQTT connect arguments
assert isinstance(config.mqtt_connect_args, tuple), "config.mqtt_connect_args is not of type tuple"
assert len(config.mqtt_connect_args) == 3, "config.mqtt_connect_args must have 3 elements"
assert isinstance(config.mqtt_connect_args[0], str), "config.mqtt_connect_args[0] is not of type str"
assert isinstance(config.mqtt_connect_args[1], int), "config.mqtt_connect_args[1] is not of type int"
assert isinstance(config.mqtt_connect_args[2], int), "config.mqtt_connect_args[2] is not of type int"
assert config.mqtt_credentials == None or isinstance(config.mqtt_credentials, tuple), "config.mqtt_credentials is not of type tuple"
assert config.mqtt_credentials == None or len(config.mqtt_credentials)==2, "config.mqtt_credentials is not of length 2 or None"
assert config.mqtt_credentials == None or isinstance(config.mqtt_credentials[0], str), "config.mqtt_credentials[0] is not of type str"
assert config.mqtt_credentials == None or isinstance(config.mqtt_credentials[1], str), "config.mqtt_credentials[1] is not of type str"

# Validate MQTT topic prefix
assert isinstance(config.mqtt_topic_prefix, str), "config.mqtt_topic_prefix is not of type str"

# Validate MQTT status broadcast interval
assert isinstance(config.mqtt_status_broadcast_interval_s, int), "config.mqtt_status_broadcast_interval_s is not of type int"

# Validate MQTT heartbeat interval
assert isinstance(config.mqtt_heartbeat_interval_s, int), "config.mqtt_heartbeat_interval_s is not of type int"

# Validate battery configuration variables
assert isinstance(config.battery_max_charge_power_W, int), "config.battery_max_charge_power_W is not of type int"
assert isinstance(config.battery_max_discharge_power_W, int), "config.battery_max_discharge_power_W is not of type int"
assert isinstance(config.battery_max_soc_charge, int), "config.battery_max_soc_charge is not of type int"
assert isinstance(config.battery_min_soc_discharge, int), "config.battery_min_soc_discharge is not of type int"
assert isinstance(config.battery_min_voltage, int), "config.battery_min_voltage is not of type int"
assert isinstance(config.battery_max_voltage, int), "config.battery_max_voltage is not of type int"

# Validate charger/inverter configuration variables
assert isinstance(config.charger_inverter_device_id, int), "config.charger_inverter_device_id is not of type int"
assert isinstance(config.charger_inverter_model_voltage, int), "config.charger_inverter_model_voltage is not of type int"
assert isinstance(config.charger_inverter_disconnect_invert_V, int), "config.charger_inverter_disconnect_invert_V is not of type int"

assert isinstance(config.min_charge_power_W, int), "config.min_charge_power_W is not of type int"
assert isinstance(config.min_invert_power_W, int), "config.min_invert_power_W is not of type int"

assert config.charger_inverter_model_voltage in [12,24,48,96], "config.charger_inverter_model_voltage bust be a valid BIC model voltage, like 48"