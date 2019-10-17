#!/usr/bin/env python

import serial
import socket
import rospy
from  sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import TimeReference
from marine_msgs.msg import NavEulerStamped
from marine_msgs.msg import Contact
from marine_msgs.msg import Heartbeat
from marine_msgs.msg import KeyValue
import datetime
import calendar
import sys
import ais.decoder

def ais_listener(logdir=None):
    position_pub = rospy.Publisher('/base/position',NavSatFix,queue_size=10)
    timeref_pub = rospy.Publisher('/base/time_reference',TimeReference,queue_size=10)
    ais_pub = rospy.Publisher('/base/ais/contacts',Contact,queue_size=10)
    ais_raw_pub = rospy.Publisher('/base/ais/raw',Heartbeat,queue_size=10)
    rospy.init_node('ais')
    input_type = rospy.get_param('/ais/input_type')
    input_address = rospy.get_param('/ais/input','')
    input_speed = rospy.get_param('/ais/input_speed',0)
    input_port = int(rospy.get_param('/ais/input_port',0))
    output_port = int(rospy.get_param('/ais/output',0))
    output_address = rospy.get_param('/ais/output_address','<broadcast>')
    
    ais_decoder = ais.decoder.AISDecoder()
    
    if logdir is not None:
        logfile = file(logdir+'.'.join(datetime.datetime.utcnow().isoformat().split(':'))+'_ais.log','w')
    else:
        logfile = None
    
    
    if input_type == 'serial':
        serial_in = serial.Serial(input_address, int(input_speed))
    else:
        udp_in = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        udp_in.bind(('',input_port))
    
    if output_port > 0:
        udp_out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        udp_out.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    else:
        udp_out = None
            
    while not rospy.is_shutdown():
        if input_type == 'serial':
            nmea_ins = (serial_in.readline(),)
            #print nmea_in
            if udp_out is not None:
                udp_out.sendto(nmea_in, (output_address,output_port))
        else:
            nmea_in,addr = udp_in.recvfrom(2048)
            #print addr, nmea_in
            nmea_ins = nmea_in.split('\n')
        now = rospy.get_rostime()
        for nmea_in in nmea_ins:
            if logfile is not None:
                logfile.write(datetime.datetime.utcnow().isoformat()+','+nmea_in+'\n')
            if nmea_in.startswith('!AIVDM'):
                ais_decoder.addNMEA(nmea_in.strip())
                msgs = ais_decoder.popMessages()
                for m in msgs:
                    if m['type'] in (1,2,3,18,19): #position reports
                        c = Contact()
                        c.header.stamp = now
                        c.mmsi = m['mmsi']
                        if 'shipname' in ais_decoder.mmsi_db[m['mmsi']]:
                            c.name = ais_decoder.mmsi_db[m['mmsi']]['shipname']
                        if 'callsign' in ais_decoder.mmsi_db[m['mmsi']]:
                            c.callsign = ais_decoder.mmsi_db[m['mmsi']]['callsign']
                        c.position.latitude = m['lat']
                        c.position.longitude = m['lon']
                        if m['course'] is not None:
                            c.cog = math.radians(m['course'])
                        else:
                            c.cog = -1
                        if m['speed'] is not None:
                            c.sog = m['speed']*0.514444 #knots to m/s
                        else:
                            c.sog = -1
                        if m['heading'] is not None:
                            c.heading = math.radians(m['heading'])
                        else:
                            c.heading = -1
                        if 'to_bow' in ais_decoder.mmsi_db[m['mmsi']]:
                            c.dimension_to_bow = ais_decoder.mmsi_db[m['mmsi']]['to_bow']
                        if 'to_stern' in ais_decoder.mmsi_db[m['mmsi']]:
                            c.dimension_to_stern = ais_decoder.mmsi_db[m['mmsi']]['to_stern']
                        if 'to_port' in ais_decoder.mmsi_db[m['mmsi']]:
                            c.dimension_to_port = ais_decoder.mmsi_db[m['mmsi']]['to_port']
                        if 'to_starboard' in ais_decoder.mmsi_db[m['mmsi']]:
                            c.dimension_to_stbd = ais_decoder.mmsi_db[m['mmsi']]['to_starboard']
                        ais_pub.publish(c)
                    raw = Heartbeat()
                    for k,v in m.iteritems():
                        raw.values.append(KeyValue(k,v))
                    ais_raw_pub.publish(raw)
                        
                        
                        
            else:
                nmea_parts = nmea_in.strip().split(',')
                if len(nmea_parts):
                    #print nmea_parts
                    try:
                        if nmea_parts[0][3:] == 'ZDA' and len(nmea_parts) >= 5:
                            tref = TimeReference()
                            tref.header.stamp = now
                            hour = int(nmea_parts[1][0:2])
                            minute = int(nmea_parts[1][2:4])
                            second = int(nmea_parts[1][4:6])
                            ms = int(float(nmea_parts[1][6:])*1000000)
                            day = int(nmea_parts[2])
                            month = int(nmea_parts[3])
                            year = int(nmea_parts[4])
                            zda = datetime.datetime(year,month,day,hour,minute,second,ms)
                            tref.time_ref = rospy.Time(calendar.timegm(zda.timetuple()),zda.microsecond*1000)
                            tref.source = 'ais'
                            timeref_pub.publish(tref)
                        if nmea_parts[0][3:] == 'GGA' and len(nmea_parts) >= 10:
                            latitude = int(nmea_parts[2][0:2])+float(nmea_parts[2][2:])/60.0
                            if nmea_parts[3] == 'S':
                                latitude = -latitude
                            longitude = int(nmea_parts[4][0:3])+float(nmea_parts[4][3:])/60.0
                            if nmea_parts[5] == 'W':
                                longitude = -longitude
                            altitude = float(nmea_parts[9])
                            nsf = NavSatFix()
                            nsf.header.stamp = now
                            nsf.header.frame_id = 'mobile_lab'
                            nsf.latitude = latitude
                            nsf.longitude = longitude
                            nsf.altitude = altitude
                            position_pub.publish(nsf)
                        if nmea_parts[0][3:] == 'HDT' and len(nmea_parts) >= 2:
                            heading = float(nmea_parts[1])
                            nes = NavEulerStamped()
                            nes.header.stamp = now
                            nes.header.frame_id = 'mobile_lab'
                            nes.orientation.heading = heading
                            orientation_pub.publish(nes)
                    except ValueError:
                        pass
        
            


if __name__ == '__main__':
    try:
        logdir = None
        argv = rospy.myargv(argv=sys.argv)
        if len(argv) > 1:
            logdir = argv[1]
        ais_listener(logdir)
    except rospy.ROSInterruptException:
        pass


