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


import datetime
import math

from marine_ais_msgs.msg import (
    AIS, Communication, DataLinkReservationBlock,
    Destination, Navigation, NavigationalStatus,
)
import marine_ais_tools.ais.decoder
from nmea_msgs.msg import Sentence
import rclpy
import rclpy.constants
import rclpy.node
import transforms3d


def markUnknown(a: AIS) -> None:
    """Set every optional field to its "not available" representation.

    AIS reports most fields with a dedicated not-available sentinel, and the
    decoder turns those into None. Each field is then populated under an
    ``if ... is not None`` guard, which makes the failure mode of a forgotten
    ``else`` subtle: the field does not stay empty, it takes the ROS default,
    and those defaults all look like real measurements. A default-constructed
    geometry_msgs Pose is position 0N 0E with an identity quaternion -- which
    is a valid heading of due east -- a default Twist is a speed of zero, and
    a default builtin_interfaces/Time is 1970-01-01T00:00:00Z.

    Three display bugs came from exactly that: heading-less contacts drawn
    pointing east, and unknown speed and course rendered as huge numbers.

    Defaulting to unknown up front inverts the failure mode. A field that
    nothing populates stays unknown, which is true, rather than becoming a
    value that looks real. Adding a field to this function is cheaper than
    remembering an ``else`` at every future call site.
    """
    # Position. NaN is this module's convention for an unavailable float.
    a.navigation.pose.position.latitude = math.nan
    a.navigation.pose.position.longitude = math.nan
    a.navigation.pose.position.altitude = math.nan

    # Orientation. A null quaternion, not the identity: consumers test
    # length2() to decide whether a heading is real (CAMP does this in
    # camp/ais/ais_contact.cpp), and the identity passes that test.
    a.navigation.pose.orientation.x = 0.0
    a.navigation.pose.orientation.y = 0.0
    a.navigation.pose.orientation.z = 0.0
    a.navigation.pose.orientation.w = 0.0

    # Velocity, carrying SOG/COG. Only x and y: linear.z stays 0.0 because
    # consumers take the length of the whole vector, and a NaN z would make
    # every speed unknown, including the ones we do know.
    a.navigation.twist.linear.x = math.nan
    a.navigation.twist.linear.y = math.nan
    # Rate of turn.
    a.navigation.twist.angular.z = math.nan
    a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_UNAVAILABLE

    # Enumerations that define their own not-available value.
    a.navigation.time_stamp = Navigation.TIME_STAMP_NOT_AVAIABLE
    a.navigation.navigational_status.status = (
        NavigationalStatus.NAVIGATIONAL_STATUS_UNDEFINED)
    a.navigation.navigational_status.special_manoeuvre = (
        NavigationalStatus.SPECIAL_MANOEUVRE_NOT_AVAILABLE)

    # Voyage/static. AIS encodes an unavailable draught as 0, which a ROS
    # consumer cannot tell from a real 0.0 m draught.
    a.static_info.static_draught = math.nan

    # NOTE: utc_time has no unknown representation -- builtin_interfaces/Time
    # has no NaN, and its default (0) is a valid instant, 1970-01-01T00:00:00Z.
    # Nothing in this workspace reads it today. If something starts to, it
    # needs an explicit validity flag in AIS.msg rather than a sentinel.


class AISParser(rclpy.node.Node):

    def __init__(self) -> None:
        super().__init__('ais_parser')

        self.ais_pub = self.create_publisher(AIS, 'messages', 10)
        self.ais_decoder = marine_ais_tools.ais.decoder.AISDecoder()
        self.nmea_sub = self.create_subscription(
            Sentence, 'nmea', self.nmeaCallback, 10)

    def nmeaCallback(self, msg: Sentence):
        if msg.sentence.startswith('!AIVDM'):
            self.ais_decoder.addNMEA(msg.sentence)
            msgs = self.ais_decoder.popMessages()
            receive_time = datetime.datetime.fromtimestamp(
                msg.header.stamp.sec +
                msg.header.stamp.nanosec /
                float(
                    rclpy.constants.S_TO_NS),
                datetime.timezone.utc)
            for m in msgs:
                a = AIS()
                markUnknown(a)
                a.header = msg.header
                a.message_id = m['message_id']
                a.repeat_indicator = m['repeat_indicator']
                a.id = m['id']

                # navigation

                if 'latitude' in m and 'longitude' in m and m[
                        'latitude'] is not None and m['longitude'] is not None:
                    a.navigation.pose.position.latitude = m['latitude']
                    a.navigation.pose.position.longitude = m['longitude']
                else:
                    a.navigation.pose.position.latitude = math.nan
                    a.navigation.pose.position.longitude = math.nan
                if 'altitude' in m and m['altitude'] is not None:
                    a.navigation.pose.position.altitude = m['altitude']
                else:
                    a.navigation.pose.position.altitude = math.nan

                if 'true_heading' in m and m['true_heading'] is not None:
                    yaw = math.radians(90.0 - m['true_heading'])
                    q = transforms3d.taitbryan.euler2quat(yaw, 0, 0)
                    a.navigation.pose.orientation.x = q[1]
                    a.navigation.pose.orientation.y = q[2]
                    a.navigation.pose.orientation.z = q[3]
                    a.navigation.pose.orientation.w = q[0]
                else:
                    # AIS true heading 511 means "not available", which the
                    # decoder turns into None. Leaving the field alone is not
                    # neutral: geometry_msgs/Quaternion defaults to the
                    # *identity* (0, 0, 0, 1), a perfectly valid orientation
                    # meaning yaw 0 -- due east in ENU. Consumers cannot tell
                    # that apart from a vessel genuinely heading east.
                    #
                    # A null quaternion is the agreed signal for "no heading":
                    # CAMP tests length2() > 0.1 before believing an
                    # orientation (camp/ais/ais_contact.cpp:45-58) and falls
                    # back to course over ground when the contact is moving,
                    # or to NaN when it is not, which draws a circle rather
                    # than a direction. Same convention as the NaN used above
                    # for unknown position and altitude.
                    a.navigation.pose.orientation.x = 0.0
                    a.navigation.pose.orientation.y = 0.0
                    a.navigation.pose.orientation.z = 0.0
                    a.navigation.pose.orientation.w = 0.0

                if 'rate_of_turn' in m:
                    if m['rate_of_turn'] is None:
                        a.navigation.twist.angular.z = math.nan
                        if 'rate_of_turn_status' in m:
                            a.navigation.rate_of_turn_status = m['rate_of_turn_status']
                        else:
                            a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_UNAVAILABLE
                    else:
                        # from deg/min clockwise to rad/sec counter clockwise
                        a.navigation.twist.angular.z = math.radians(
                            -m['rate_of_turn'] / 60.0)
                        a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_VALID
                else:
                    a.navigation.twist.angular.z = math.nan
                    a.navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_UNAVAILABLE

                if 'sog' in m and m['sog'] is not None and 'cog' in m and m['cog'] is not None:
                    sog_meters_per_second = m['sog'] * 0.514444
                    cog_ros = math.radians(90.0 - m['cog'])
                    a.navigation.twist.linear.x = math.cos(
                        cog_ros) * sog_meters_per_second
                    a.navigation.twist.linear.y = math.sin(
                        cog_ros) * sog_meters_per_second
                else:
                    a.navigation.twist.linear.x = math.nan
                    a.navigation.twist.linear.y = math.nan

                if 'navigational_status' in m:
                    a.navigation.navigational_status.status = m['navigational_status']
                else:
                    a.navigation.navigational_status.status = (
                        NavigationalStatus.NAVIGATIONAL_STATUS_UNDEFINED
                    )

                a.navigation.position_accuracy_high = 'position_accuracy' in m and m[
                    'position_accuracy'] == 1

                if 'time_stamp' in m:
                    a.navigation.time_stamp = m['time_stamp']
                else:
                    a.navigation.time_stamp = Navigation.TIME_STAMP_NOT_AVAIABLE

                if 'special_manoeuvre_indicator' in m:
                    a.navigation.navigational_status.special_manoeuvre = m[
                        'special_manoeuvre_indicator']

                a.navigation.raim_in_use = 'raim_flag' in m and m['raim_flag'] == 1

                if 'position_fixing_device_type' in m:
                    a.navigation.position_fixing_device_type = m['position_fixing_device_type']

                a.navigation.barometric_altitude = 'altitude_sensor' in m and m[
                    'altitude_sensor'] == 1

                if 'position_latency' in m:
                    a.navigation.position_latency = m['position_latency']
                a.navigation.position_latency = 1

                a.navigation.assigned_mode = 'assigned_mode_flag' in m and m[
                    'assigned_mode_flag'] == 1

                # Static

                if 'ais_version' in m:
                    a.static_info.ais_version = m['ais_version']

                if 'imo_number' in m:
                    a.static_info.imo_number = m['imo_number']

                if 'callsign' in m:
                    a.static_info.callsign = m['callsign']

                if 'name' in m:
                    a.static_info.name = m['name']

                if 'ship_and_cargo_type' in m:
                    a.static_info.ship_and_cargo_type = m['ship_and_cargo_type']

                if 'reference_to_bow_distance' in m:
                    a.static_info.reference_to_bow_distance = m['reference_to_bow_distance']

                if 'reference_to_stern_distance' in m:
                    a.static_info.reference_to_stern_distance = m['reference_to_stern_distance']

                if 'reference_to_port_distance' in m:
                    a.static_info.reference_to_port_distance = m['reference_to_port_distance']

                if 'reference_to_starboard_distance' in m:
                    a.static_info.reference_to_starboard_distance = (
                        m['reference_to_starboard_distance']
                    )

                if 'maximum_present_static_draught' in m and m[
                        'maximum_present_static_draught'] is not None:
                    a.static_info.static_draught = m['maximum_present_static_draught']

                if 'dte' in m:
                    a.static_info.dte_ready = m['dte'] == 0
                else:
                    a.static_info.dte_ready = False

                # Voyage

                eta_parts_available = False

                year = receive_time.year
                month = receive_time.month
                if 'eta_month' in m:
                    if m['eta_month'] > 0:
                        month == m['eta_month']
                        if month < receive_time.month:  # next year?
                            year += 1
                        eta_parts_available = True
                day = receive_time.day
                if 'eta_day' in m:
                    if m['eta_day'] > 0:
                        day = m['eta_day']
                        eta_parts_available = True
                hour = 0
                if 'eta_hour' in m:
                    if m['eta_hour'] < 24:
                        hour = m['eta_hour']
                        eta_parts_available = True
                minute = 0
                if 'eta_minute' in m:
                    if m['eta_minute'] < 60:
                        minute = m['eta_minute']
                        eta_parts_available = True

                if eta_parts_available:
                    try:
                        eta_dt = datetime.datetime(
                            year, month, day, hour, minute,
                            tzinfo=datetime.timezone.utc,
                        )
                        a.voyage.estimated_time_of_arrival = (
                            rclpy.time.Time(
                                seconds=eta_dt.timestamp()
                            ).to_msg()
                        )
                    except ValueError:
                        s = "Can't decode eta from:\n" + str(m)
                        self.get_logger().warn(s)

                if 'destination' in m:
                    a.voyage.destination = m['destination']

                # Addressed

                if 'destination_id' in m:
                    d = Destination()
                    d.id = m['destination_id']
                    if 'sequence_number' in m:
                        d.sequence_number = m['sequence_number']
                    a.addressed.destinations.append(d)

                if 'destination_id1' in m:
                    d = Destination()
                    d.id = m['destination_id1']
                    if 'sequence_number_for_id1' in m:
                        d.sequence_number = m['sequence_number_for_id1']
                    if 'message_id1_1' in m:
                        d.requested_message_id = m['message_id1_1']
                    if 'slot_offset_1_1' in m:
                        d.slot_offset = m['slot_offset_1_1']
                    a.addressed.destinations.append(d)
                    if 'message_id1_2' in m:
                        d = Destination()
                        d.id = m['destination_id1']
                        d.requested_message_id = m['message_id1_2']
                        if 'slot_offset_1_2' in m:
                            d.slot_offset = m['slot_offset_1_2']
                        a.addressed.destinations.append(d)

                if 'destination_id2' in m:
                    d = Destination()
                    d.id = m['destination_id2']
                    if 'sequence_number_for_id2' in m:
                        d.sequence_number = m['sequence_number_for_id2']
                    if 'message_id2_1' in m:
                        d.requested_message_id = m['message_id2_1']
                    if 'slot_offset_2_1' in m:
                        d.slot_offset = m['slot_offset_2_1']
                    a.addressed.destinations.append(d)

                if 'destination_id3' in m:
                    d = Destination()
                    d.id = m['destination_id3']
                    if 'sequence_number_for_id3' in m:
                        d.sequence_number = m['sequence_number_for_id3']
                    a.addressed.destinations.append(d)

                if 'destination_id4' in m:
                    d = Destination()
                    d.id = m['destination_id4']
                    if 'sequence_number_for_id4' in m:
                        d.sequence_number = m['sequence_number_for_id4']
                    a.addressed.destinations.append(d)

                if 'destination_id_a' in m:
                    d = Destination()
                    d.id = m['destination_id_a']
                    if 'offset_a' in m:
                        d.slot_offset = m['offset_a']
                    if 'increment_a' in m:
                        d.increment = m['increment_a']
                    a.addressed.destinations.append(d)

                if 'destination_id_b' in m:
                    d = Destination()
                    d.id = m['destination_id_b']
                    if 'offset_b' in m:
                        d.slot_offset = m['offset_b']
                    if 'increment_b' in m:
                        d.increment = m['increment_b']
                    a.addressed.destinations.append(d)

                a.addressed.retransmitted = 'retransmit_flag' in m and m['retransmit_flag'] == 1

                # Binary

                if 'binary_data' in m:
                    b = m['binary_data']
                    if 'application_identifier' in b:
                        ai = b['application_identifier']
                        if 'designated_area_code' in ai:
                            a.binary.designated_area_code = ai['designated_area_code']
                        if 'function_identifier' in ai:
                            a.binary.function_identifier = ai['function_identifier']
                    if 'unstructured_data' in b:
                        a.binary.unstructured = True
                        a.binary.data = b['unstructured_data']
                    if 'differential_correction_data' in b:
                        a.binary.data = b['differential_correction_data']
                    if 'application_data' in b:
                        a.binary.data = b['application_data']

                # Class B

                a.class_b.cs_unit = 'class_b_unit_flag' in m and m['class_b_unit_flag'] == 1

                a.class_b.display_equipped = 'class_b_display_flag' in m and m[
                    'class_b_display_flag'] == 1

                a.class_b.dsc_equipped = 'class_b_dsc_flag' in m and m['class_b_dsc_flag'] == 1

                a.class_b.whole_band = 'class_b_band_flag' in m and m['class_b_band_flag'] == 1

                a.class_b.message_22_frequency_management = 'class_b_message_22_flag' in m and m[
                    'class_b_message_22_flag'] == 1

                if 'part_number' in m:
                    a.class_b.part_number = m['part_number']

                if 'vendor_id' in m:
                    vid = m['vendor_id']
                    if 'manufacturers_id' in vid:
                        a.class_b.manufacturer_id = vid['manufacturers_id']
                    if 'unit_model_code' in vid:
                        a.class_b.unit_model_code = vid['unit_model_code']
                    if 'unit_serial_number' in vid:
                        a.class_b.unit_serial_number = vid['unit_serial_number']

                # Datalink

                for n in ('1', '2', '3', '4'):

                    if 'offset_number_' + n in m:
                        dlrb = DataLinkReservationBlock()
                        dlrb.offset_number = m['offset_number_' + n]

                        if 'number_of_slots_' + n in m:
                            dlrb.number_of_slots = m['number_of_slots_' + n]
                        if 'timeout_' + n in m:
                            dlrb.timeout = rclpy.duration.Duration(
                                seconds=60 * m['timeout_' + n]).to_msg()
                        if 'increment_' + n in m:
                            dlrb.increment = m['increment_' + n]
                        a.data_link_reservation_blocks.append(dlrb)

                # Aids to Navigation

                if 'type_of_aids_to_navigation' in m:
                    a.aton.type = m['type_of_aids_to_navigation']

                if 'name_of_aids_navigation' in m:
                    a.static_info.name = m['name_of_aids_navigation']

                if 'name_of_aids_to_navigation_extension' in m:
                    a.static_info.name += m['name_of_aids_to_navigation_extension']

                if 'off_position_indicator' in m:
                    a.aton.off_position = m['off_position_indicator'] == 1

                if 'aton_status' in m:
                    a.aton.status = m['aton_status']

                if 'virtual_aton_flag' in m:
                    a.aton.virtual_aton = m['virtual_aton_flag'] == 1

                # Communication

                if m['message_id'] in (1, 2, 4, 11):
                    a.communication.state = Communication.COMMUNICATION_STATE_SOTDMA
                elif m['message_id'] == 3:
                    a.communication.state = Communication.COMMUNICATION_STATE_ITDMA
                elif 'communication_state_selector_flag' in m:
                    a.communication.state = m['communication_state_selector_flag']

                if 'sotdma_sync_state' in m:
                    a.communication.sync_state = m['sotdma_sync_state']

                if 'sotdma_slot_timeout' in m:
                    a.communication.sotdma_slot_timeout = m['sotdma_slot_timeout']

                if 'sotdma_received_stations' in m:
                    a.communication.sotdma_received_stations = m['sotdma_received_stations']

                if 'sotdma_slot_number' in m:
                    a.communication.sotdma_slot_number = m['sotdma_slot_number']

                if 'sotdma_utc_hour' in m:
                    a.communication.sotdma_utc_hour = m['sotdma_utc_hour']

                if 'sotdma_utc_minute' in m:
                    a.communication.sotdma_utc_minute = m['sotdma_utc_minute']

                if 'sotdma_slot_offset' in m:
                    a.communication.sotdma_slot_offset = m['sotdma_slot_offset']

                if 'itdma_sync_state' in m:
                    a.communication.sync_state = m['itdma_sync_state']

                if 'itdma_slot_increment' in m:
                    a.communication.itdma_slot_increment = m['itdma_slot_increment']

                if 'itdma_number_of_slots' in m:
                    a.communication.itdma_number_of_slots = m['itdma_number_of_slots']

                if 'itdma_keep_flag' in m:
                    a.communication.itdma_keep = m['itdma_keep_flag'] == 1

                # Channel Management

                if 'utc_time' in m and m['utc_time'] is not None:
                    try:
                        a.utc_time = rclpy.time.Time(
                            seconds=m['utc_time'].timestamp()).to_msg()
                    except TypeError:
                        self.get_logger().error(
                            'utc_time: ' + str(m['utc_time']))

                a.long_range_transmission_control = 'long_range_transmission_control' in m and m[
                    'long_range_transmission_control'] == 1

                if 'safety_related_text' in m:
                    a.safety_related_text = m['safety_related_text']

                if 'channel' in m:
                    a.channel = m['channel']

                if 'nmea_payload' in m:
                    a.nmea_payload = m['nmea_payload']

                try:
                    self.ais_pub.publish(a)
                except Exception as e:
                    self.get_logger().error(str(e))


def main(args=None):
    rclpy.init(args=args)
    parser = AISParser()
    rclpy.spin(parser)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
