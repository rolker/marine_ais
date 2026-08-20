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
import math

from geographic_msgs.msg import GeoPointStamped
from geometry_msgs.msg import Point32, Polygon
from marine_ais_msgs.msg import AIS, AISContact, Navigation, Static
import rclpy
import rclpy.node


# Row-major indices of the variance diagonal in a 6x6 ROS covariance array,
# ordered (x, y, z, rot_x, rot_y, rot_z) for a pose and
# (vx, vy, vz, wx, wy, wz) for a twist.
COV_XX = 0
COV_YY = 7
COV_ZZ = 14
COV_RR = 21
COV_PP = 28
COV_AA = 35


def isValidQuaternion(q, tolerance=1.0e-3):
    """Return True if q is a usable unit quaternion.

    This is a sanity check only, NOT a heading-present test. ais_parser now
    zeroes the orientation when no heading was reported, so a null quaternion
    does fail this check -- but do not rely on that: the ROS default for
    geometry_msgs/Quaternion is the IDENTITY, so any orientation that reaches
    us untouched still passes happily while carrying no heading at all. Use
    Navigation.heading_valid to answer "was a heading reported"; use this to
    reject a quaternion that is numerically unusable.
    """
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    return abs(norm - 1.0) < tolerance


def hasUsableHeading(navigation):
    """Return True if this report carried a real true heading.

    AIS sends 511 for "not available"; the decoder nulls it and the parser
    then clears heading_valid AND zeroes the orientation. Both signals say the
    same thing: the flag for consumers that read it, the null quaternion for
    those already testing the orientation's length. Without either, an absent
    heading arrives as the identity quaternion and reads as due east.
    """
    return (navigation.heading_valid and
            isValidQuaternion(navigation.pose.orientation))


def calculatePolygon(static: Static):

    # REP 103: In relation to a body the standard is:
    # x forward, y left, z up
    bow = Point32()
    bow.x = float(static.reference_to_bow_distance)
    width = static.reference_to_port_distance + \
        static.reference_to_starboard_distance
    half_width = width / 2.0
    bow.y = half_width - static.reference_to_port_distance
    length = float(static.reference_to_bow_distance + static.reference_to_stern_distance)
    port_pointy_start = Point32()
    port_pointy_start.x = bow.x - (length * 0.1)
    port_pointy_start.y = float(static.reference_to_port_distance)
    starboard_pointy_start = Point32()
    starboard_pointy_start.x = port_pointy_start.x
    starboard_pointy_start.y = float(-static.reference_to_starboard_distance)
    aft_port = Point32()
    aft_port.x = float(-static.reference_to_stern_distance)
    aft_port.y = float(static.reference_to_port_distance)
    aft_starboard = Point32()
    aft_starboard.x = aft_port.x
    aft_starboard.y = float(-static.reference_to_starboard_distance)

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

        # Uncertainty model. AIS transmits a single position-accuracy bit, not a
        # covariance, so these turn that bit (plus what the standard says about
        # each field's resolution) into the covariances consumers need. Defaults
        # are deliberately conservative: a consumer that trusts an AIS fix more
        # than it should is the dangerous direction.
        #
        # ITU-R M.1371 defines the accuracy bit as high => better than 10 m
        # (a differentially-corrected fix) and low => worse than 10 m. The
        # defaults below sit inside and outside that bound respectively; the low
        # case has no upper bound in the standard, so 25 m is a judgement call
        # and is a parameter precisely so it can be revised against observation.
        self.declare_parameter('position_sigma_high', 5.0)
        self.declare_parameter('position_sigma_low', 25.0)
        # SOG is transmitted at 0.1 kn and COG at 0.1 degree resolution, but
        # real-world agreement is far coarser than the quantisation.
        self.declare_parameter('speed_sigma', 0.5)
        self.declare_parameter('heading_sigma', math.radians(5.0))
        self.declare_parameter('rate_of_turn_sigma', 0.02)
        # Stand-in variance for quantities AIS never reports (altitude, roll,
        # pitch) or that are absent from this particular message. Large-but-
        # finite rather than -1: consumers weighting by 1/variance stay
        # numerically well-behaved, and anything checking a threshold still
        # sees "effectively unknown".
        self.declare_parameter('unknown_variance', 1.0e6)

        self.ais_message_sub = self.create_subscription(
            AIS, 'messages', self.aisCallback, 10)

    def fillCovariances(self, contact: AISContact, navigation: Navigation):
        """Populate contact.covariance and contact.twist.covariance.

        AIS transmits a single position-accuracy bit rather than a covariance,
        so this maps that bit -- and the presence or absence of the optional
        heading and rate-of-turn fields -- onto the two 6x6 covariance arrays
        that ROS consumers expect. Fields AIS never carries (altitude, roll,
        pitch) and fields absent from this particular report get
        `unknown_variance` rather than zero: a zero variance asserts a
        perfectly known quantity, which is the opposite of the truth and would
        let a consumer weight garbage infinitely.
        """
        unknown = self.get_parameter('unknown_variance').value

        if navigation.position_accuracy_high:
            position_sigma = self.get_parameter('position_sigma_high').value
        else:
            position_sigma = self.get_parameter('position_sigma_low').value
        position_variance = position_sigma * position_sigma

        covariance = [0.0] * 36
        covariance[COV_XX] = position_variance
        covariance[COV_YY] = position_variance
        covariance[COV_ZZ] = unknown
        covariance[COV_RR] = unknown
        covariance[COV_PP] = unknown
        if hasUsableHeading(navigation):
            heading_sigma = self.get_parameter('heading_sigma').value
            covariance[COV_AA] = heading_sigma * heading_sigma
        else:
            covariance[COV_AA] = unknown
        contact.covariance = covariance

        twist_covariance = [0.0] * 36
        # ais_parser writes NaN into linear x/y when SOG or COG was absent, so
        # the velocity is not merely uncertain, it is missing.
        if (math.isnan(navigation.twist.linear.x) or
                math.isnan(navigation.twist.linear.y)):
            twist_covariance[COV_XX] = unknown
            twist_covariance[COV_YY] = unknown
        else:
            speed_sigma = self.get_parameter('speed_sigma').value
            speed_variance = speed_sigma * speed_sigma
            twist_covariance[COV_XX] = speed_variance
            twist_covariance[COV_YY] = speed_variance
        twist_covariance[COV_ZZ] = unknown
        twist_covariance[COV_RR] = unknown
        twist_covariance[COV_PP] = unknown
        if navigation.rate_of_turn_status == Navigation.RATE_OF_TURN_VALID:
            rate_of_turn_sigma = self.get_parameter('rate_of_turn_sigma').value
            twist_covariance[COV_AA] = rate_of_turn_sigma * rate_of_turn_sigma
        else:
            twist_covariance[COV_AA] = unknown
        contact.twist.covariance = twist_covariance

    def aisCallback(self, msg: AIS):
        self.get_logger().debug('msg: ' + str(msg))
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
                self.fillCovariances(self.contacts[msg.id], msg.navigation)
                self.get_logger().debug(
                    'publishing contact: ' + str(self.contacts[msg.id]))

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
