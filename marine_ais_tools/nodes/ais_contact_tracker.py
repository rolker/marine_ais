#!/usr/bin/env python3

import rospy
from marine_ais_msgs.msg import AIS, AISContact
from geometry_msgs.msg import Polygon, Point32
from geographic_msgs.msg import GeoPointStamped

import copy

rospy.init_node('ais_contact_tracker')

contacts_pub = rospy.Publisher('contacts', AISContact, queue_size=10)

# added to track Mesobot, which uses an AIS beacon that transmits as an AtoN
# setting frame_id to id(mmsi) as a quick solution
aton_pub = rospy.Publisher('atons', GeoPointStamped, queue_size=10)

contacts = {}

def calculatePolygon(static):

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


def aisCallback(msg):
    if msg.message_id in (1,2,3,5,9,18,19,24):
        if not msg.id in contacts:
            contacts[msg.id] = AISContact()
            contacts[msg.id].id = msg.id
        if msg.message_id in (5,24): # static/voyage
            if msg.message_id == 5:
                contacts[msg.id].static_info = copy.deepcopy(msg.static_info)
                contacts[msg.id].voyage = copy.deepcopy(msg.voyage)
            else:
                if msg.class_b.part_number == 0: # 24A
                    contacts[msg.id].static_info.name = msg.static_info.name
                    contacts[msg.id].footprint = calculatePolygon(contacts[msg.id].static_info)
                else: # 24B
                    contacts[msg.id].static_info.callsign = msg.static_info.callsign
                    contacts[msg.id].static_info.ship_and_cargo_type = msg.static_info.ship_and_cargo_type
                    contacts[msg.id].static_info.reference_to_bow_distance = msg.static_info.reference_to_bow_distance
                    contacts[msg.id].static_info.reference_to_stern_distance = msg.static_info.reference_to_stern_distance
                    contacts[msg.id].static_info.reference_to_port_distance = msg.static_info.reference_to_port_distance
                    contacts[msg.id].static_info.reference_to_starboard_distance = msg.static_info.reference_to_starboard_distance
                    contacts[msg.id].footprint = calculatePolygon(contacts[msg.id].static_info)
        if msg.message_id in (1,2,3,9,18,19):
            contacts[msg.id].header = msg.header
            contacts[msg.id].position_message_id = msg.message_id
            contacts[msg.id].pose = msg.navigation.pose
            contacts[msg.id].twist.twist = msg.navigation.twist
            contacts[msg.id].navigational_status = msg.navigation.navigational_status
            # todo, figure out covariances

            contacts_pub.publish(contacts[msg.id])
    if msg.message_id == 21:
        #Aid to Navigation
        aton = GeoPointStamped()
        aton.header.stamp = msg.header.stamp
        aton.header.frame_id = msg.id
        aton.position = msg.navigation.pose.position
        aton_pub.publish(aton)

ais_message_sub = rospy.Subscriber('messages', AIS, aisCallback)            

rospy.spin()
