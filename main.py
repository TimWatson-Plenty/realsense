from realsense_device_manager import*
from helpers import*
import paho.mqtt.client as paho
import ssl
import boto3
from botocore.exceptions import NoCredentialsError


c = rs.config()
c.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 6)
c.enable_stream(rs.stream.color, 1280, 720, rs.format.rgb8, 6)

device_manager = DeviceManager(rs.context(), c)
device_manager.enable_all_devices()

device_manager.load_settings_json_all('settings.json')
print(enumerate_connected_devices(rs.context()))

def on_connect(client, userdata, flags, rc):
    print("Connection returned result: " + str(rc) )
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe("#" , 1 )

def on_message(client, userdata, msg):
    print("topic: "+msg.topic)
    print("payload: "+str(msg.payload))
    device_manager.save_color_image('6CD14603005A')
    #upload_to_aws('color_img.jpg', 'timsnewbucket', 'color_img')

#def on_log(client, userdata, level, msg):
#    print(msg.topic+" "+str(msg.payload))

mqttc = paho.Client()
mqttc.on_connect = on_connect
mqttc.on_message = on_message
#mqttc.on_log = on_log

awshost = "data.iot.us-west-1.amazonaws.com"
awsport = 8883
clientId = "myThingName"
thingName = "myThingName"
caPath = "aws-iot-rootCA.crt"
certPath = "cert.pem"
keyPath = "privkey.pem"

mqttc.tls_set(caPath, certfile=certPath, keyfile=keyPath, cert_reqs=ssl.CERT_REQUIRED, tls_version=ssl.PROTOCOL_TLSv1_2, ciphers=None)

mqttc.connect(awshost, awsport, keepalive=60)
mqttc.loop_forever()


