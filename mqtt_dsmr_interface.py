from decimal import Decimal
import paho.mqtt.client as mqtt
import json
from loguru import logger
import dsmr_parser

class SetEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, dsmr_parser.objects.CosemObject):
            d = obj.values[0]

            if isinstance(d, dict) and "unit" in d and d["unit"] == None:
                del d["unit"]
            return d
        if isinstance(obj, Decimal):
            return float(obj)
        if isinstance(obj, dsmr_parser.objects.MBusObject):
            d = dict()
            d["unit"] = obj.unit
            d["datetime"] = obj.datetime.isoformat()
            d["value"] = obj.value
            return d
        if isinstance(obj, dsmr_parser.objects.MbusDevice):
            return { k:v for (k,v) in obj }
        return super(SetEncoder, self).default(obj)
        
class MqttBroadcaster:
    def __init__(self, connect_args, credentials, topic):
        # Create a client instance
        self.__client = mqtt.Client()
        self.__client.on_connect = self.on_connect
        self.__client.on_publish = self.on_publish
        self.__client.on_message = self.on_message

        self.on_power_request = None
        self.__topic = topic

        # Start the network loop
        self.__client.loop_start()
        
        if credentials != None:
            self.__client.username_pw_set(*credentials)

        self.__client.connect(*connect_args)


    # Define callback functions for connecting, publishing, and receiving messages
    def on_connect(self, client, userdata, flags, rc):
        logger.info(f"Connected with result code {rc}")

    def on_publish(self, client, userdata, mid):
        logger.info(f"Message published with ID {mid}: {userdata}")

    def on_message(self, client, userdata, msg):
        logger.info(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

    def broadcast(self, message):
        d = {} 
    #    for (key, entry) in message:
   #         d[key] = entry.values[0]
        mapped = { k:v for (k,v) in message}
        json_msg = json.dumps(mapped, cls=SetEncoder)
        self.__client.publish(self.__topic, json_msg)
    