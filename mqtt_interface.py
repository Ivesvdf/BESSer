import json
import paho.mqtt.client as mqtt

from loguru import logger


class SetEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, set):
            return list([x.name for x in obj])
        return super(SetEncoder, self).default(obj)


class MqttInterface:
    def __init__(self, connect_args, credentials, prefix):
        # Create a client instance
        self.__client = mqtt.Client()
        self.__client.on_connect = self.on_connect
        self.__client.on_publish = self.on_publish
        self.__client.on_message = self.on_message

        self.on_power_request = None
        self.on_min_soc = None
        self.on_max_soc = None
        self.on_heartbeat = None
        self.on_Ki = None
        self.on_Kp = None
        self.on_Kd = None
        self.prefix = prefix

        # Start the network loop
        self.__client.loop_start()

        if credentials != None:
            self.__client.username_pw_set(*credentials)

        self.__client.connect(*connect_args)

        self.__charge_discharge_topic = f"{self.prefix}/charge_discharge_request"
        self.__status_topic = f"{self.prefix}/status"
        self.__min_soc_topic = f"{self.prefix}/min_soc"
        self.__max_soc_topic = f"{self.prefix}/max_soc"
        self.__Ki_topic = f"{self.prefix}/charger_inverter_PID_Ki"
        self.__Kp_topic = f"{self.prefix}/charger_inverter_PID_Kp"
        self.__Kd_topic = f"{self.prefix}/charger_inverter_PID_Kd"

    # Define callback functions for connecting, publishing, and receiving messages
    def on_connect(self, client: mqtt.Client, userdata, flags, rc):
        logger.info(f"Connected with result code {rc}")
        # Subscribe to a topic

        def subscribe(topic):
            logger.info(f"Subscribing to MQTT topic {topic}")
            client.subscribe(topic)

        subscribe(self.__charge_discharge_topic)
        subscribe(self.__status_topic)
        subscribe(self.__min_soc_topic)
        subscribe(self.__max_soc_topic)
        subscribe(self.__Ki_topic)
        subscribe(self.__Kp_topic)
        subscribe(self.__Kd_topic)

    def on_publish(self, client, userdata, mid):
        logger.info(f"Message published with ID {mid}: {userdata}")

    def __handle_int(self, topic, fun, msg):
        try:
            if fun != None:
                fun(int(float(msg.payload.decode())))
        except ValueError:
            logger.error(f"Could not process received {topic} instruction")
        except:
            logger.exception(f"Invalid {topic} instruction")

    def __handle_float(self, topic, fun, msg):
        try:
            if fun != None:
                fun(float(msg.payload.decode()))
        except ValueError:
            logger.error(f"Could not process received {topic} instruction")
        except:
            logger.exception(f"Invalid {topic} instruction")

    def on_message(self, client, userdata, msg):
        logger.info(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

        if msg.topic == self.__charge_discharge_topic:
            self.__handle_int(self.__charge_discharge_topic, self.on_power_request, msg)
            if self.on_heartbeat != None:
                self.on_heartbeat()
        elif msg.topic == self.__min_soc_topic:
            self.__handle_int(self.__min_soc_topic, self.on_min_soc, msg)

        elif msg.topic == self.__max_soc_topic:
            self.__handle_int(self.__max_soc_topic, self.on_max_soc, msg)

        elif msg.topic == self.__Ki_topic:
            self.__handle_float(self.__Ki_topic, self.on_Ki, msg)
        elif msg.topic == self.__Kp_topic:
            self.__handle_float(self.__Kp_topic, self.on_Kp, msg)
        elif msg.topic == self.__Kd_topic:
            self.__handle_float(self.__Kd_topic, self.on_Kd, msg)

    def broadcast_status(self, status):
        self.__client.publish(f"{self.prefix}/status", json.dumps(status, cls=SetEncoder))

    def broadcast_debug(self, status):
        self.__client.publish(f"{self.prefix}/debug", json.dumps(status, cls=SetEncoder))
