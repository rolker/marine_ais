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

"""Tests for footprint construction in ais_contact_tracker.

The footprint is what lets a downstream consumer (the ais_layer costmap
plugin) paint a vessel at its true size. The two traps covered here:

* Class A dimensions arrive ONLY in message 5. If that branch never builds
  the footprint, a 200 m tanker paints as the consumer's default ~10 m
  circle -- under-painting, which is the dangerous direction.
* ITU-R M.1371 defines A = B = C = D = 0 as "dimensions not available".
  Building a polygon from that yields a degenerate outline with every
  vertex at the reference point -- downstream, a radius-0 hull that paints
  nothing at all.
"""

from marine_ais_msgs.msg import AIS

from marine_ais_tools.ais_contact_tracker import AisContactTracker

import pytest

import rclpy


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def tracker(ros):
    node = AisContactTracker()
    yield node
    node.destroy_node()


MMSI = 366000001


def makeStaticAndVoyage(bow=150, stern=50, port=15, starboard=15):
    msg = AIS()
    msg.message_id = AIS.STATIC_AND_VOYAGE_RELATED_DATA
    msg.id = MMSI
    msg.static_info.reference_to_bow_distance = bow
    msg.static_info.reference_to_stern_distance = stern
    msg.static_info.reference_to_port_distance = port
    msg.static_info.reference_to_starboard_distance = starboard
    return msg


def makeStaticDataReport(part_number, bow=20, stern=10, port=4, starboard=4):
    msg = AIS()
    msg.message_id = AIS.STATIC_DATA_REPORT
    msg.id = MMSI
    msg.class_b.part_number = part_number
    if part_number == 0:  # 24A carries only the name
        msg.static_info.name = 'TEST VESSEL'
    else:  # 24B carries the dimensions
        msg.static_info.reference_to_bow_distance = bow
        msg.static_info.reference_to_stern_distance = stern
        msg.static_info.reference_to_port_distance = port
        msg.static_info.reference_to_starboard_distance = starboard
    return msg


def test_message_5_builds_the_class_a_footprint(tracker):
    tracker.aisCallback(makeStaticAndVoyage(bow=150, stern=50,
                                            port=15, starboard=15))

    footprint = tracker.contacts[MMSI].footprint
    assert len(footprint.points) == 6
    xs = [p.x for p in footprint.points]
    ys = [p.y for p in footprint.points]
    # A 200 m vessel must span 200 m bow to stern and 30 m beam to beam.
    assert max(xs) == pytest.approx(150.0)
    assert min(xs) == pytest.approx(-50.0)
    assert max(ys) == pytest.approx(15.0)
    assert min(ys) == pytest.approx(-15.0)


def test_message_5_with_no_dimensions_leaves_the_footprint_empty(tracker):
    # A = B = C = D = 0 is "dimensions not available", not a zero-size ship.
    tracker.aisCallback(makeStaticAndVoyage(bow=0, stern=0,
                                            port=0, starboard=0))

    # No polygon at all: the consumer falls back to its default radius,
    # instead of receiving a degenerate outline that paints nothing.
    assert len(tracker.contacts[MMSI].footprint.points) == 0


def test_message_24a_without_dimensions_leaves_the_footprint_empty(tracker):
    # 24A (name only) arrives before 24B (dimensions): there is nothing to
    # build a polygon from yet.
    tracker.aisCallback(makeStaticDataReport(part_number=0))

    assert len(tracker.contacts[MMSI].footprint.points) == 0


def test_message_24b_builds_the_class_b_footprint(tracker):
    tracker.aisCallback(makeStaticDataReport(part_number=0))
    tracker.aisCallback(makeStaticDataReport(part_number=1, bow=20, stern=10,
                                             port=4, starboard=4))

    footprint = tracker.contacts[MMSI].footprint
    assert len(footprint.points) == 6
    xs = [p.x for p in footprint.points]
    ys = [p.y for p in footprint.points]
    assert max(xs) == pytest.approx(20.0)
    assert min(xs) == pytest.approx(-10.0)
    assert max(ys) == pytest.approx(4.0)
    assert min(ys) == pytest.approx(-4.0)


def test_message_24b_with_no_dimensions_keeps_a_previous_footprint(tracker):
    # Dimensions reported once, then a later report says "not available":
    # the ship did not shrink, so the known footprint is kept.
    tracker.aisCallback(makeStaticDataReport(part_number=1, bow=20, stern=10,
                                             port=4, starboard=4))
    tracker.aisCallback(makeStaticDataReport(part_number=1, bow=0, stern=0,
                                             port=0, starboard=0))

    assert len(tracker.contacts[MMSI].footprint.points) == 6
