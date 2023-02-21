import paho.mqtt.client as mqtt
import json
from loguru import logger
import enum

from battery import AlarmFlags, ProtectionFlags, RequestFlags

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
        self.on_heartbeat = None
        self.prefix = prefix

        # Start the network loop
        self.__client.loop_start()

        
        if credentials != None:
            self.__client.username_pw_set(*credentials)

        self.__client.connect(*connect_args)

        self.__charge_discharge_topic = f"{self.prefix}/charge_discharge_request"
        self.__status_topic = f"{self.prefix}/status"

    # Define callback functions for connecting, publishing, and receiving messages
    def on_connect(self, client, userdata, flags, rc):
        print(f"Connected with result code {rc}")
        # Subscribe to a topic
        client.subscribe(self.__charge_discharge_topic)
        client.subscribe(self.__status_topic)

        client.will_set(self.__status_topic, payload=None, qos=0, retain=True)

    def on_publish(self, client, userdata, mid):
        print(f"Message published with ID {mid}: {userdata}")

    def on_message(self, client, userdata, msg):
        print(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

        if msg.topic == f"{self.prefix}/charge_discharge_request":
            if self.on_power_request != None:
                try:
                    self.on_power_request(int(msg.payload.decode()))
                except ValueError:
                    logger.error("Could not process received charge/discharge instruction")
                except:
                    logger.exception("Invalid charge/discharge instruction")
            if self.on_heartbeat != None:
                self.on_heartbeat()

    def broadcast_status(self, status):
        self.__client.publish(f"{self.prefix}/status", json.dumps(status, cls=SetEncoder))
    


