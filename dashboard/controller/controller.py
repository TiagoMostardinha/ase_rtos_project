import paho.mqtt.client as mqtt
import threading
import time

sendfile = False

def on_connect(client, userdata, flags, rc, properties):
    print("Connected with result code "+str(rc))
    client.subscribe("boat/file")

def on_message(client, userdata, msg):
    global sendfile

    message = msg.payload.decode('utf-8')

    if message == "END":
        sendfile = False
        client.publish("boat/in", " ")

    
    if sendfile:
        with open('data.txt', 'a') as file:
            file.write(message+"\n")
    
    if message == "START":
        sendfile = True
        
    

        
            

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect("172.30.0.2", 1883, 60)
    threading.Thread(target=client.loop_forever).start()

    realtime = False
    turn_motor = False

    print("Commands:")
    print("r - Realtime")
    print("f - Send file")
    print("o - Motor on/off")
    print("q - Boat off")
    print("a - Servo Left")
    print("d - Servo Right")
    print("s - Servo Middle")



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
            client.publish("boat/in", "servo_left")
        elif cmd == 'd':
            client.publish("boat/in", "servo_right")
        elif cmd == 's':
            client.publish("boat/in", "servo_middle")

        time.sleep(0.1)

if __name__ == '__main__':
    main()
