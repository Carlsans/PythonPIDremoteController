import os


class NetworkConfiguration:
    def __init__(self):
        self.esp8266_ip = os.environ.get('YOGURT_ESP_IP', "192.168.0.4") # Put your ESP IP address here !
        self.esp8266_port = int(os.environ.get('YOGURT_ESP_PORT', 5000))
        # Port this program listens on. The ESP sends to the same port number
        # it listens on, so this defaults to esp8266_port. Overriding it (and
        # the two values above) lets the automated tests run against a fake
        # ESP on localhost without ever reaching the real device.
        self.listen_port = int(os.environ.get('YOGURT_LISTEN_PORT', self.esp8266_port))
