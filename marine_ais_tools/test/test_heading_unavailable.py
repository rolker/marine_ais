# Copyright 2026 University of New Hampshire
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the University of New Hampshire nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
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

"""Unknown AIS heading must not be reported as a valid orientation.

AIS encodes "true heading not available" as 511, which the decoder turns into
None. Leaving geometry_msgs/Quaternion untouched in that case is not neutral:
it defaults to the identity (0, 0, 0, 1), a valid orientation meaning yaw 0 --
due east in ENU. Every heading-less contact then renders as pointing east, and
no consumer can tell it from a vessel genuinely on an easterly heading.

The convention is a null quaternion; CAMP checks length2() > 0.1 before
believing an orientation (camp/ais/ais_contact.cpp:45-58).
"""

from marine_ais_tools.ais_parser import AISParser

from nmea_msgs.msg import Sentence

import pytest

import rclpy


# Class B position report (message 18) from the live Portsmouth feed. Class B
# equipment commonly reports no true heading.
CLASS_B_NO_HEADING = '!AIVDM,1,1,,B,B5NgbN00Bng68RV:4kiUOwuUkP06,0*69'
# Class A position report (message 1) carrying a real heading.
CLASS_A_WITH_HEADING = '!AIVDM,1,1,,B,15ND1BP000Jt?`@H`jJsUa820<0b,0*02'


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _parse(sentence):
    """Feed one NMEA sentence through the parser, returning published AIS msgs."""
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
    return published


def _length2(q):
    return q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w


def test_missing_heading_yields_a_null_orientation(ros):
    messages = _parse(CLASS_B_NO_HEADING)
    assert messages, 'sentence did not decode to an AIS message'
    assert not messages[0].navigation.heading_valid
    orientation = messages[0].navigation.pose.orientation
    assert _length2(orientation) < 0.1, (
        'unknown heading must be a null quaternion, not the identity that '
        'geometry_msgs/Quaternion defaults to -- the identity reads as due east'
    )


def test_present_heading_still_yields_a_usable_orientation(ros):
    messages = _parse(CLASS_A_WITH_HEADING)
    assert messages, 'sentence did not decode to an AIS message'
    orientation = messages[0].navigation.pose.orientation
    assert _length2(orientation) > 0.9, 'a known heading must survive as a real orientation'
