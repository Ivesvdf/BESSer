import paho.mqtt.client as mqtt
import json
from loguru import logger

class SetEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, set):
            return list([x.name for x in obj])
        return super(SetEncoder, self).default(obj)
        
class MqttInterface:
    def __init__(self, config): # connect_args, credentials, prefix):
        # Create a client instance
        self.__client = mqtt.Client()
        self.__client.on_connect = self.on_connect
        self.__client.on_publish = self.on_publish
        self.__client.on_message = self.on_message

        self._mqtt_config = config['mqtt']
        self._prefix = self._mqtt_config['topic_prefix']

        # Start the network loop
        self.__client.loop_start()

        if 'credentials' in self._mqtt_config:
            self.__client.username_pw_set(**self._mqtt_config['credentials'])

        self.__status_topic = f"{self._prefix}/status"

        self._extra_subscriptions = []
        self._handlers = dict() # topic : (cast function, handler_function)

    def add_subscriptions(self, subs):
        for sub in subs:
            self._extra_subscriptions.append(sub)

    def connect(self):
        self.__client.connect(**self._mqtt_config['connection'])

    # Define callback functions for connecting, publishing, and receiving messages
    def on_connect(self, client:mqtt.Client, userdata, flags, rc):
        logger.info(f"Connected with result code {rc}")
        # Subscribe to a topic

        def subscribe(topic):
            logger.info(f"Subscribing to MQTT topic {topic}")
            client.subscribe(topic)

        subscribe(self.__status_topic)

        for sub in self._extra_subscriptions:
            (topic, cast, handler) = sub
            full_topic = f"{self._prefix}/{topic}"
            
            data = (cast, handler)
            self._handlers[full_topic] = data

            subscribe(full_topic)

    def on_publish(self, client, userdata, mid):
        logger.debug(f"Message published with ID {mid}: {userdata}")

    def __handle_handler(self, msg, cast, handler):
        try:
            if handler != None:
                handler(cast(msg.payload.decode()))
        except ValueError:
            logger.error(f"Could not process received {msg.topic} instruction")
        except:
            logger.exception(f"Invalid {msg.topic} instruction")

    def on_message(self, client, userdata, msg):
        logger.info(f"Received message on topic {msg.topic}: {msg.payload.decode()}")

        if msg.topic in self._handlers:
            (cast, handler) = self._handlers[msg.topic]
            self.__handle_handler(msg, cast, handler)

    def broadcast_status(self, status):
        self.__client.publish(f"{self._prefix}/status", json.dumps(status, cls=SetEncoder))
    
    def broadcast_debug(self, status):
        self.__client.publish(f"{self._prefix}/debug", json.dumps(status, cls=SetEncoder))