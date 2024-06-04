import paho.mqtt.client as mqtt
import threading
import time


def on_connect(client, userdata, flags, rc, properties):
    print("Connected with result code "+str(rc))
    client.subscribe("boat/file")

def on_message(client, userdata, msg):
    if message == "START":
        with open('data.txt', 'a') as file:
            while message != "END":
                message = msg.payload.decode('utf-8')
                file.write(message)
        
            

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("localhost", 1883, 60)
    threading.Thread(target=client.loop_forever).start()

    realtime = False
    turn_motor = False

    print("Commands:")
    print("r - Realtime")
    print("f - Send file")
    print("o - Motor on/off")
    print("q - Boat off")
    print("a - Servo 0")
    print("d - Servo 2")
    print("s - Servo 1")



    while True:
        cmd = input()
        if cmd == 'r':
            if realtime:
                client.publish("boat/in", "realtime_off")
                realtime = False
            else:
                client.publish("boat/in", "realtime")
                realtime = True
        elif cmd == 'f':
            client.publish("boat/in", "sendfile")
        elif cmd == 'o':
            if turn_motor:
                client.publish("boat/in", "motoroff")
                turn_motor = False
            else:
                client.publish("boat/in", "motor")
                turn_motor = True
        elif cmd == 'q':
            client.publish("boat/in", "boatoff")
        elif cmd == 'a':
            client.publish("boat/in", "servo0")
        elif cmd == 'd':
            client.publish("boat/in", "servo2")
        elif cmd == 's':
            client.publish("boat/in", "servo1")

        time.sleep(0.1)

if __name__ == '__main__':
    main()