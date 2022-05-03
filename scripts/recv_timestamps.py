#!/usr/bin/python3
import time
import serial

import rospy
from lora_messages.msg import lora_distance

def millis():
    return int(round(time.time() * 1000))

ser = serial.Serial(
        port='/dev/ttyACM0', 
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)

def print_delta(node_timestamp,tag):
    timestamp = millis()
    time_delta = []
    drone_to_node_time_delta = timestamp - int(node_timestamp)
    time_delta.clear()
    time_delta.append(tag)
    time_delta.append(drone_to_node_time_delta)
    print(time_delta)

while True:
    bytes_to_read = ser.inWaiting()
    buf = ""
    if bytes_to_read > 0:
        bytes_to_read = ser.inWaiting()
        buf = ser.readline()
        last_activity = millis()
    message = [ x.decode('utf8') for x in buf.split()]
    if len(message) == 2:
        print(message)
        if message[0]=='Node0Timestamp:':
            print_delta(message[1],"DroneNode0TimeDelta")
        if message[0]=='Node1Timestamp:':
            print_delta(message[1],"DroneNode1TimeDelta")
        if message[0]=='Node2Timestamp:':
            print_delta(message[1],"DroneNode2TimeDelta")
