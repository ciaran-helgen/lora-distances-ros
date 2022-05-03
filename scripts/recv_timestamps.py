#!/usr/bin/python3
import time
import serial

import rospy
from lora_messages.msg import lora_distance

dist0 = lora_distance()
dist1 = lora_distance()
dist2 = lora_distance()

speed_of_light = 299792458

rospy.init_node('lora_distance_publisher', anonymous=False)
    
pub0 = rospy.Publisher("/distance0", lora_distance, queue_size=1)
pub1 = rospy.Publisher("/distance1", lora_distance, queue_size=1)
pub2 = rospy.Publisher("/distance1", lora_distance, queue_size=1)

def millis():
    return int(round(time.time() * 1000))

ser = serial.Serial(
        port='/dev/ttyACM0', 
        baudrate=9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1)

def calc_distance(node_timestamp,tag):
    timestamp = millis()
    time_delta = []
    drone_to_node_time_delta = timestamp - int(node_timestamp)
    time_delta.clear()
    time_delta.append(tag)
    time_delta.append(drone_to_node_time_delta)
    print(time_delta)
    beacon_dist = time_delta*speed_of_light
    return beacon_dist

if __name__ == '__main__':

    rate = rospy.Rate(1)

while not rospy.is_shutdown():
    try:
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
                beacon_dist0 = calc_distance(message[1],"DroneNode0TimeDelta")
            if message[0]=='Node1Timestamp:':
                beacon_dist1 = calc_distance(message[1],"DroneNode1TimeDelta")
            if message[0]=='Node2Timestamp:':
                beacon_dist2 = calc_distance(message[1],"DroneNode1TimeDelta")

            now = rospy.Time.now()

            dist0.header.stamp = now
            dist1.header.stamp = now
            dist2.header.stamp = now

            dist0.distance = beacon_dist0
            dist1.distance = beacon_dist1
            dist2.distance = beacon_dist2

            dist0.raw_data = message[0]

            pub0.publish(dist0)
            pub1.publish(dist1)
            pub2.publish(dist2)


            rate.sleep()

    except rospy.ROSInterruptException:
        pass
