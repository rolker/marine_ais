#!/usr/bin/env python3

import rospy
from nmea_msgs.msg import Sentence
from rosgraph_msgs.msg import Clock
import datetime
import time

rospy.init_node('nmea_replay')

input_filename = rospy.get_param("~filename")

frame_id = rospy.get_param('~frame_id','nmea')

rate = rospy.get_param("~rate", 1.0)

publish_clock = rospy.get_param('~clock', False)

nmea_pub = rospy.Publisher('nmea', Sentence, queue_size=20)

if publish_clock:
  clock_publisher = rospy.Publisher('/clock', Clock, queue_size = 5)

data_start_time = None
clock_period = datetime.timedelta(seconds=0.2)

wallclock_start = datetime.datetime.utcnow()
next_clock_time = wallclock_start

for line in open(input_filename).readlines():
  timestring,nmea = line.split(',',1)
  datatime = datetime.datetime.fromisoformat(timestring)
  if data_start_time is None:
    data_start_time = datatime

  while nmea is not None:  
    now = datetime.datetime.utcnow()
    rosnow = data_start_time+datetime.timedelta(seconds=(now-wallclock_start).total_seconds()*rate)

    if publish_clock and next_clock_time <= now:
      rosclock_time = data_start_time+datetime.timedelta(seconds=(next_clock_time-wallclock_start).total_seconds()*rate)
      c = Clock(rospy.Time.from_sec(rosclock_time.timestamp()))
      clock_publisher.publish(c)
      next_clock_time += clock_period

    if datatime <= rosnow:
      sentence = Sentence()
      sentence.header.frame_id = frame_id
      sentence.header.stamp = rospy.Time.from_sec(datatime.timestamp())
      sentence.sentence = nmea
      nmea_pub.publish(sentence)
      nmea = None
      break
    sleeptime = (datatime - rosnow).total_seconds()/rate
    if publish_clock:
      clocksleeptime = (next_clock_time - now).total_seconds()
      sleeptime = min(clocksleeptime, sleeptime)
    time.sleep(sleeptime)


