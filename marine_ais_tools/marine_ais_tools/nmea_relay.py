#!/usr/bin/env python3

import serial
import socket
import rospy
from nmea_msgs.msg import Sentence
import datetime

class SerialReader:
    def __init__(self, address, speed):
        self.serial_in = serial.Serial(address, speed)

    def readlines(self):
        nmea_in = self.serial_in.readline()
        return (nmea_in,)

class UDPReader:
    def __init__(self, port):
        self.udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_in.settimeout(0.1)
        self.udp_in.bind(('', port))

    def readlines(self):
        try:
            nmea_in = self.udp_in.recv(2048)
            nmea_ins = nmea_in.decode('utf-8').split('\n')
        except socket.timeout:
            return []
        ret = []
        for n in nmea_ins:
            ret.append(n.strip())
        return ret

class TCPReader:
    def __init__(self, address, port):
        self.tcp_in = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_in.settimeout(0.1)
        self.tcp_in.connect((address, port))
        self.leftovers = ''

    def readlines(self):
        try:
            nmea_in = self.tcp_in.recv(256)
            nmea_ins = (self.leftovers+nmea_in.decode('utf-8')).split('\n')
            if len(nmea_ins):
                self.leftovers = nmea_ins[-1]
                ret = []
                for n in nmea_ins[:-1]:
                    ret.append(n.strip())
                return ret
        except socket.timeout:
            pass
        return []

def nmea_listener(logdir=None):
    rospy.init_node('nmea_relay')

    nmea_pub = rospy.Publisher('nmea', Sentence, queue_size=20)

    input_type = rospy.get_param('~input_type')
    input_address = rospy.get_param('~input_address','')
    input_speed = int(rospy.get_param('~input_speed',0))
    input_port = int(rospy.get_param('~input_port',0))

    output_port = int(rospy.get_param('~output',0))
    output_address = rospy.get_param('~output_address','<broadcast>')
    frame_id = rospy.get_param("~frame_id",'nmea')
    if rospy.has_param("~log_directory"):
        logdir = rospy.get_param("~log_directory")
    
    if logdir is not None and logdir != "":
        logfile = open(logdir+'ais_'+'.'.join(datetime.datetime.utcnow().isoformat().split(':'))+'.log','w')
    else:
        logfile = None
    
    
    if input_type == 'serial':
        reader = SerialReader(input_address, input_speed)
    elif input_type == 'tcp':
        reader = TCPReader(input_address, input_port)
    else:
        reader = UDPReader(input_port)
    
    if output_port > 0:
        udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    else:
        udp_out = None
            
    while not rospy.is_shutdown():
        nmea_ins = reader.readlines()
        now = rospy.get_rostime()

        for nmea in nmea_ins:
            if udp_out is not None:
                udp_out.sendto(nmea, (output_address,output_port))

            if logfile is not None:
                logfile.write(datetime.datetime.fromtimestamp(now.to_sec()).isoformat()+','+nmea+'\n')
                logfile.flush()
            if len(nmea) > 0:
                sentence = Sentence()
                sentence.header.stamp = now
                sentence.header.frame_id = frame_id
                sentence.sentence = nmea
                nmea_pub.publish(sentence)

if __name__ == '__main__':
    try:
        nmea_listener()
    except rospy.ROSInterruptException:
        pass


