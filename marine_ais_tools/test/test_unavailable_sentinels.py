"""Every AIS "not available" sentinel must survive as unknown, not as a value.

AIS encodes most fields with a dedicated not-available sentinel. The decoder
turns those into None, and each field is populated under an
``if ... is not None`` guard -- so a forgotten ``else`` does not leave the
field empty, it leaves the ROS default, and every one of those defaults reads
as a real measurement: position 0N 0E, an identity quaternion (a heading of
due east), a speed of zero, a 1970 timestamp.

Three display bugs came from that, so this sweeps the whole set: one sentence
with every sentinel set, one with none of them, and one message type that
carries no position at all.

The payloads are real AIVDM sentences, bit-encoded so the decoder is exercised
rather than bypassed.
"""

import math

from marine_ais_msgs.msg import AIS, Navigation, NavigationalStatus

from marine_ais_tools.ais_parser import AISParser, markUnknown

from nmea_msgs.msg import Sentence

import pytest

import rclpy

# Message 1, every position field at its not-available sentinel: rate of turn
# -128, SOG 1023, longitude 181, latitude 91, COG 3600, true heading 511,
# time stamp 60, navigational status 15.
MSG1_ALL_UNAVAILABLE = '!AIVDM,1,1,,A,15MwqhOP?w<tSF0l4Q@>4?wp0000,0*40'
# The same message with every one of those fields carrying a real value.
MSG1_ALL_AVAILABLE = '!AIVDM,1,1,,A,15MwqhP01srtA@0H`RL3Q2lt0000,0*6A'
# Message 5 -- static and voyage data, which carries no position or velocity.
MSG5_STATIC_ONLY = (
    '!AIVDM,1,1,,A,55Mwqhh000000000000000000000000'
    '0000000000000000000000000000000000000000000,0*5D')


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _parse(sentence):
    node = AISParser()
    published = []
    node.ais_pub.publish = published.append
    msg = Sentence()
    msg.sentence = sentence
    msg.header.stamp.sec = 1787245290
    try:
        node.nmeaCallback(msg)
    finally:
        node.destroy_node()
    assert published, f'sentence did not decode: {sentence}'
    return published[0]


def _quaternion_length2(q):
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w


def test_every_sentinel_reports_unknown(ros):
    a = _parse(MSG1_ALL_UNAVAILABLE)
    position = a.navigation.pose.position
    assert math.isnan(position.latitude)
    assert math.isnan(position.longitude)
    assert _quaternion_length2(a.navigation.pose.orientation) < 0.1, \
        'unknown heading must be a null quaternion; the identity reads as due east'
    assert math.isnan(a.navigation.twist.linear.x)
    assert math.isnan(a.navigation.twist.linear.y)
    assert math.isnan(a.navigation.twist.angular.z)
    assert a.navigation.rate_of_turn_status == Navigation.RATE_OF_TURN_UNAVAILABLE
    assert a.navigation.time_stamp == Navigation.TIME_STAMP_NOT_AVAIABLE
    assert a.navigation.navigational_status.status == \
        NavigationalStatus.NAVIGATIONAL_STATUS_UNDEFINED


def test_available_values_are_not_clobbered_by_the_unknown_defaults(ros):
    """The control case: defaulting to unknown must not swallow real data."""
    a = _parse(MSG1_ALL_AVAILABLE)
    position = a.navigation.pose.position
    assert position.latitude == pytest.approx(43.05, abs=1e-4)
    assert position.longitude == pytest.approx(-70.72, abs=1e-4)
    assert _quaternion_length2(a.navigation.pose.orientation) > 0.9
    assert not math.isnan(a.navigation.twist.linear.x)
    assert not math.isnan(a.navigation.twist.linear.y)
    assert a.navigation.rate_of_turn_status != Navigation.RATE_OF_TURN_UNAVAILABLE
    assert a.navigation.time_stamp == 30
    assert a.navigation.navigational_status.status == 0


def test_fields_a_message_type_never_carries_stay_unknown(ros):
    """A message with no position must not report one. This is the shape a
    forgotten guard takes: nothing writes the field, so it keeps the ROS
    default -- which for a Pose is 0N 0E, off the coast of Africa."""
    a = _parse(MSG5_STATIC_ONLY)
    assert math.isnan(a.navigation.pose.position.latitude)
    assert math.isnan(a.navigation.pose.position.longitude)
    assert math.isnan(a.navigation.twist.linear.x)


def test_unavailable_draught_is_not_reported_as_zero_metres(ros):
    """AIS encodes an unavailable draught as 0, which a consumer cannot tell
    from a real 0.0 m draught once it is in a float field. This one has no
    ``else`` on its guard, so it relies entirely on the unknown defaults."""
    a = _parse(MSG5_STATIC_ONLY)
    assert math.isnan(a.static_info.static_draught)


def test_vertical_velocity_stays_zero_not_unknown():
    """linear.z must NOT default to NaN. Consumers take the length of the whole
    linear vector to get speed over ground, so a NaN z would make every speed
    unknown -- including the speeds that were reported."""
    a = AIS()
    markUnknown(a)
    assert a.navigation.twist.linear.z == 0.0
