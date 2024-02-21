#!/usr/bin/env python3

import rclpy
import rclpy.node

from marine_ais_msgs.msg import AIS, AISContact, Static
from geometry_msgs.msg import Polygon, Point32
from geographic_msgs.msg import GeoPointStamped

import copy


def calculatePolygon(static: Static):

    # REP 103: In relation to a body the standard is:
    # x forward, y left, z up
    bow = Point32()
    bow.x = static.reference_to_bow_distance
    width = static.reference_to_port_distance + static.reference_to_starboard_distance
    half_width = width/2.0
    bow.y = half_width-static.reference_to_port_distance
    length = static.reference_to_bow_distance + static.reference_to_stern_distance
    port_pointy_start = Point32()
    port_pointy_start.x = bow.x-(length*0.1)
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
        
        self.ais_message_sub = self.create_subscription(AIS, 'messages', self.aisCallback, 10)


    def aisCallback(self, msg: AIS):
        if msg.message_id in (1,2,3,5,9,18,19,24):
            if not msg.id in self.contacts:
                self.contacts[msg.id] = AISContact()
                self.contacts[msg.id].id = msg.id
            if msg.message_id in (5,24): # static/voyage
                if msg.message_id == 5:
                    self.contacts[msg.id].static_info = copy.deepcopy(msg.static_info)
                    self.contacts[msg.id].voyage = copy.deepcopy(msg.voyage)
                else:
                    if msg.class_b.part_number == 0: # 24A
                        self.contacts[msg.id].static_info.name = msg.static_info.name
                        self.contacts[msg.id].footprint = calculatePolygon(self.contacts[msg.id].static_info)
                    else: # 24B
                        self.contacts[msg.id].static_info.callsign = msg.static_info.callsign
                        self.contacts[msg.id].static_info.ship_and_cargo_type = msg.static_info.ship_and_cargo_type
                        self.contacts[msg.id].static_info.reference_to_bow_distance = msg.static_info.reference_to_bow_distance
                        self.contacts[msg.id].static_info.reference_to_stern_distance = msg.static_info.reference_to_stern_distance
                        self.contacts[msg.id].static_info.reference_to_port_distance = msg.static_info.reference_to_port_distance
                        self.contacts[msg.id].static_info.reference_to_starboard_distance = msg.static_info.reference_to_starboard_distance
                        self.contacts[msg.id].footprint = calculatePolygon(self.contacts[msg.id].static_info)
            if msg.message_id in (1,2,3,9,18,19):
                self.contacts[msg.id].header = msg.header
                self.contacts[msg.id].position_message_id = msg.message_id
                self.contacts[msg.id].pose = msg.navigation.pose
                self.contacts[msg.id].twist.twist = msg.navigation.twist
                self.contacts[msg.id].navigational_status = msg.navigation.navigational_status
                # todo, figure out covariances

                self.contacts_pub.publish(self.contacts[msg.id])
        if msg.message_id == 21:
            #Aid to Navigation
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


