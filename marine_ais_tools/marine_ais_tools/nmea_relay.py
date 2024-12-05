#!/usr/bin/env python3

import os

import serial
import socket
import rclpy
import rclpy.node
from nmea_msgs.msg import Sentence
import datetime

class SerialReader:
    def __init__(self, address, speed):
        self.serial_in = serial.Serial(address, speed)

    def readlines(self):
        nmea_in = self.serial_in.readline()
        return [nmea_in.decode('utf-8').strip()]

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

class NMEARelay(rclpy.node.Node):
    def __init__(self) -> None:
        super().__init__('nmea_relay')

        self.nmea_pub = self.create_publisher(Sentence, 'nmea', 10)

        self.declare_parameter('frame_id', 'nmea')
        frame_id = self.get_parameter('frame_id').value

        self.declare_parameter('input_type', 'udp')
        input_type = self.get_parameter('input_type').value

        self.declare_parameter('input_address', '127.0.0.1')
        input_address = self.get_parameter('input_address').value

        self.declare_parameter('input_speed', 9600)
        input_speed = int(self.get_parameter('input_speed').value)

        self.declare_parameter('input_port', 10110)
        input_port = int(self.get_parameter('input_port').value)

        self.declare_parameter('output_address', '127.0.0.1')
        output_address = self.get_parameter('output_address').value

        self.declare_parameter('output_port', 0)
        output_port = int(self.get_parameter('output_port').value)

        self.declare_parameter('log_directory', '')
        log_directory = self.get_parameter('log_directory').value

        if log_directory is not None and log_directory != '':
            logfile = open(os.path.join(log_directory, 'ais_'+'.'.join(datetime.datetime.utcnow().isoformat().split(':'))+'.log'),'w')
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

        while True:
            nmea_ins = reader.readlines()
            now = self.get_clock().now()

            for nmea in nmea_ins:
                if udp_out is not None:
                    udp_out.sendto(nmea.encode('utf-8'), (output_address, output_port))

                if logfile is not None:
                    logfile.write(datetime.datetime.fromtimestamp(now.sec.to_time()).isoformat() + ',' + nmea + '\n')
                    logfile.flush()
                if len(nmea) > 0:
                    sentence = Sentence()
                    sentence.header.stamp = now.to_msg()
                    sentence.header.frame_id = frame_id
                    sentence.sentence = nmea
                    self.nmea_pub.publish(sentence)

def main(args=None):
    rclpy.init(args=args)
    relay = NMEARelay()
    rclpy.spin(relay)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
