#!/usr/bin/env python3

import rclpy
import rclpy.node
import rclpy.constants

from nmea_msgs.msg import Sentence
from rosgraph_msgs.msg import Clock
import datetime
import time

class NMEAReplay(rclpy.node.Node):
  def __init__(self):
    super().__init__('nmea_replay')

    self.declare_parameter("filename", "")
    self.input_filename = self.get_parameter("filename").value

    self.declare_parameter('frame_id', 'nmea')
    self.frame_id = self.get_parameter('frame_id').value

    self.declare_parameter('rate', 1.0)
    self.rate = self.get_parameter('rate').value

    self.declare_parameter('clock', False)
    self.publish_clock = self.get_parameter('clock').value

    self.nmea_pub = self.create_publisher(Sentence, 'nmea', 20)

    if self.publish_clock:
      self.clock_publisher = self.create_publisher(Clock, '/clock', 5)


    self.run_timer = self.create_timer(0.0, self.run)


  def run(self):
    self.run_timer.cancel()
    
    data_start_time = None
    clock_period = datetime.timedelta(seconds=0.2)

    wallclock_start = datetime.datetime.utcnow()
    next_clock_time = wallclock_start

    for line in open(self.input_filename).readlines():
      timestring,nmea = line.split(',',1)
      datatime = datetime.datetime.fromisoformat(timestring)
      if data_start_time is None:
        data_start_time = datatime

      while nmea is not None:  
        now = datetime.datetime.utcnow()
        rosnow = data_start_time+datetime.timedelta(seconds=(now-wallclock_start).total_seconds()*self.rate)

        if self.publish_clock and next_clock_time <= now:
          rosclock_time = data_start_time+datetime.timedelta(seconds=(next_clock_time-wallclock_start).total_seconds()*self.rate)
          seconds = int(rosclock_time.timestamp())
          nanoseconds = rclpy.constants.S_TO_NS*(rosclock_time.timestamp()-seconds)
          c = Clock()
          c.clock = rclpy.time.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
          self.clock_publisher.publish(c)
          next_clock_time += clock_period

        if datatime <= rosnow:
          sentence = Sentence()
          sentence.header.frame_id = self.frame_id
          seconds = int(datatime.timestamp())
          nanoseconds = rclpy.constants.S_TO_NS*(datatime.timestamp()-seconds)
          sentence.header.stamp = rclpy.time.Time(seconds=seconds, nanoseconds=nanoseconds).to_msg()
          sentence.sentence = nmea
          self.nmea_pub.publish(sentence)
          nmea = None
          break
        sleeptime = (datatime - rosnow).total_seconds()/self.rate
        if self.publish_clock:
          clocksleeptime = (next_clock_time - now).total_seconds()
          sleeptime = min(clocksleeptime, sleeptime)
        time.sleep(sleeptime)


def main(args=None):
    rclpy.init(args=args)
    replay = NMEAReplay()
    rclpy.spin(replay)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
