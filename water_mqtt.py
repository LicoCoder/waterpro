import spidev 
from numpy import interp 
import time 
import RPi.GPIO as GPIO
import Adafruit_DHT
from sensor import MCP3004
import paho.mqtt.client as mqtt
import json

def get_airmsg():
    # Set sensor type : Options are DHT11,DHT22 or AM2302
    sensor = Adafruit_DHT.DHT11
    # Set GPIO sensor is connected to
    gpio = 18
    print("go in airmsg")
    time.sleep(2)
    humidity, temperature = Adafruit_DHT.read_retry(sensor, gpio)
    print("start airmsg!")
    if humidity is not None and temperature is not None:
        airmsg = 'Temp={0:0.1f}*C Humidity={1:0.1f}%'.format(temperature, humidity)
        print('Temp={0:0.1f}*C Humidity={1:0.1f}%'.format(temperature, humidity))       
        client.publish('watering_ljy/raspberry', payload=airmsg, qos=0, retain=False)
    else:
        print('Failed to get reading. Try again!')

def water_pro():
    # 自己接入的那个 pin 口
    watering_channel = 19
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)

    mcp = MCP3004(bus=0,addr=0,vref=3.3)
    mcp._spi.max_speed_hz = 2106000
    while True:
        output = mcp.read(0) # Reading from CH0
        print(output)
        # 数值越大越干燥 对于植物来说，一般大于400则可以进行浇水。
        if output > 400:
            # 进行控制开关水，每次浇水10秒钟
            GPIO.setup(watering_channel, GPIO.OUT)
            GPIO.output(watering_channel, GPIO.HIGH)
            time.sleep(10)
            GPIO.setup(watering_channel, GPIO.OUT)
            GPIO.output(watering_channel, GPIO.LOW)
        output = mcp.read(0)
        output = interp(output, [0, 1023], [100, 0])
        output = int(output)
        print("Moisture:", output)
        client.publish('watering_ljy/raspberry', payload="waterpro", qos=0, retain=False)
        time.sleep(5)

# 自己接入的那个 pin 口
watering_channel = 19
GPIO.cleanup()
GPIO.setmode(GPIO.BCM)

# 当服务器响应的时候，会回调这个函数
def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    # 订阅 raspberry/topic 主题
    client.subscribe("wateringljy/windows")
    
# 回调函数，当收到消息时，触发该函数
def on_message(client, userdata, msg):
    print(f"{msg.topic} {msg.payload}")
    payload_json = json.loads(msg.payload)
    print(payload_json['msg'])
    if payload_json['msg'] == "get_airmsg":
        get_airmsg()
    elif payload_json['msg'] == "water_pro":
        water_pro()
    elif payload_json['msg'] == "open":
        GPIO.setup(watering_channel, GPIO.OUT)
        GPIO.output(watering_channel, GPIO.HIGH)
        client.publish('watering_ljy/raspberry', payload="wateropen", qos=0, retain=False)
    elif payload_json['msg'] == "close":
        GPIO.setup(watering_channel, GPIO.OUT)
        GPIO.output(watering_channel, GPIO.LOW)
        client.publish('watering_ljy/raspberry', payload="waterclose", qos=0, retain=False)

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

# 设置遗嘱消息（立遗嘱），当树莓派断电，或者网络出现异常中断时，发送遗嘱消息往这个 topic
client.will_set('watering_ljy/raspberry', b'{"status": "Off"}')

# 创建连接，三个参数分别为 broker 地址，broker 端口号，保活时间 broker.emqx.io 是免费的mqtt服务器，如果是自己搭建的服务器需要填上自己的ip地址
#当服务器响应的时候 会回调 on_connect 方法
client.connect("broker.emqx.io", 1883, 60)

# 设置网络循环堵塞，在调用 disconnect() 或程序崩溃前，不会主动结束程序
client.loop_forever()


