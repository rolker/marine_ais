#!/usr/bin/env python3

# Copyright (c) 2016-2020, Roland Arsenault
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import copy

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Point32, Polygon
from marine_ais_msgs.msg import AIS, AISContact, Static
import rclpy
import rclpy.node


def calculatePolygon(static: Static):

    # REP 103: In relation to a body the standard is:
    # x forward, y left, z up
    bow = Point32()
    bow.x = static.reference_to_bow_distance
    width = static.reference_to_port_distance + \
        static.reference_to_starboard_distance
    half_width = width / 2.0
    bow.y = half_width - static.reference_to_port_distance
    length = static.reference_to_bow_distance + static.reference_to_stern_distance
    port_pointy_start = Point32()
    port_pointy_start.x = bow.x - (length * 0.1)
    port_pointy_start.y = static.reference_to_port_distance
    starboard_pointy_start = Point32()
    starboard_pointy_start.x = port_pointy_start.x
    starboard_pointy_start.y = -static.reference_to_starboard_distance
    aft_port = Point32()
    aft_port.x = -static.reference_to_stern_distance
    aft_port.y = static.reference_to_port_distance
    aft_starboard = Point32()
    aft_starboard.x = aft_port.x
    aft_starboard.y = -static.reference_to_starboard_distance

    footprint = Polygon()
    footprint.points.append(bow)
    footprint.points.append(port_pointy_start)
    footprint.points.append(aft_port)
    footprint.points.append(aft_starboard)
    footprint.points.append(starboard_pointy_start)
    footprint.points.append(bow)
    return footprint


class AisContactTracker(rclpy.node.Node):

    def __init__(self):
        super().__init__('ais_contact_tracker')

        self.contacts_pub = self.create_publisher(AISContact, 'contacts', 10)

        # added to track Mesobot, which uses an AIS beacon that transmits as an AtoN
        # setting frame_id to id(mmsi) as a quick solution
        self.aton_pub = self.create_publisher(GeoPointStamped, 'atons', 10)

        self.contacts = {}

        self.ais_message_sub = self.create_subscription(
            AIS, 'messages', self.aisCallback, 10)

    def aisCallback(self, msg: AIS):
        if msg.message_id in (1, 2, 3, 5, 9, 18, 19, 24):
            if msg.id not in self.contacts:
                self.contacts[msg.id] = AISContact()
                self.contacts[msg.id].id = msg.id
            if msg.message_id in (5, 24):  # static/voyage
                if msg.message_id == 5:
                    self.contacts[msg.id].static_info = copy.deepcopy(
                        msg.static_info)
                    self.contacts[msg.id].voyage = copy.deepcopy(msg.voyage)
                else:
                    if msg.class_b.part_number == 0:  # 24A
                        self.contacts[msg.id].static_info.name = msg.static_info.name
                        self.contacts[msg.id].footprint = calculatePolygon(
                            self.contacts[msg.id].static_info)
                    else:  # 24B
                        self.contacts[msg.id].static_info.callsign = msg.static_info.callsign
                        self.contacts[msg.id].static_info \
                            .ship_and_cargo_type = \
                            msg.static_info.ship_and_cargo_type
                        self.contacts[msg.id].static_info \
                            .reference_to_bow_distance = \
                            msg.static_info.reference_to_bow_distance
                        self.contacts[msg.id].static_info \
                            .reference_to_stern_distance = \
                            msg.static_info.reference_to_stern_distance
                        self.contacts[msg.id].static_info \
                            .reference_to_port_distance = \
                            msg.static_info.reference_to_port_distance
                        self.contacts[msg.id].static_info \
                            .reference_to_starboard_distance = \
                            msg.static_info \
                            .reference_to_starboard_distance
                        self.contacts[msg.id].footprint = calculatePolygon(
                            self.contacts[msg.id].static_info)
            if msg.message_id in (1, 2, 3, 9, 18, 19):
                self.contacts[msg.id].header = msg.header
                self.contacts[msg.id].position_message_id = msg.message_id
                self.contacts[msg.id].pose = msg.navigation.pose
                self.contacts[msg.id].twist.twist = msg.navigation.twist
                self.contacts[msg.id].navigational_status = msg.navigation.navigational_status
                # todo, figure out covariances

                self.contacts_pub.publish(self.contacts[msg.id])
        if msg.message_id == 21:
            # Aid to Navigation
            aton = GeoPointStamped()
            aton.header.stamp = msg.header.stamp
            aton.header.frame_id = str(msg.id)
            aton.position = msg.navigation.pose.position
            self.aton_pub.publish(aton)


def main(args=None):
    rclpy.init(args=args)
    tracker = AisContactTracker()
    rclpy.spin(tracker)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
