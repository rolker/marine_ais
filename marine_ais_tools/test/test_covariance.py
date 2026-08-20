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

"""Tests for the AISContact covariance population in ais_contact_tracker."""

import math

from marine_ais_msgs.msg import AISContact, Navigation

from marine_ais_tools.ais_contact_tracker import (
    AisContactTracker,
    COV_AA,
    COV_PP,
    COV_RR,
    COV_XX,
    COV_YY,
    COV_ZZ,
    hasUsableHeading,
    isValidQuaternion,
)

import pytest

import rclpy


UNKNOWN = 1.0e6


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


def makeNavigation(accuracy_high=True, sog_cog=True, heading=True,
                   rate_of_turn_valid=True):
    navigation = Navigation()
    navigation.position_accuracy_high = accuracy_high
    if sog_cog:
        navigation.twist.linear.x = 3.0
        navigation.twist.linear.y = 4.0
    else:
        navigation.twist.linear.x = math.nan
        navigation.twist.linear.y = math.nan
    if heading:
        navigation.pose.orientation.w = 1.0
        navigation.heading_valid = True
    if rate_of_turn_valid:
        navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_VALID
    else:
        navigation.rate_of_turn_status = Navigation.RATE_OF_TURN_UNAVAILABLE
    return navigation


def test_default_orientation_is_identity_not_zero():
    # The trap this whole flag exists for: the ROS default for a Quaternion is
    # the IDENTITY, so an orientation ais_parser never touched is a perfectly
    # valid rotation meaning "due east" in ENU. A quaternion sanity check
    # therefore CANNOT answer "was a heading reported".
    navigation = Navigation()
    assert navigation.pose.orientation.w == 1.0
    assert isValidQuaternion(navigation.pose.orientation)


def test_usable_heading_requires_the_explicit_flag():
    navigation = Navigation()
    # Untouched by the parser: valid quaternion, but no heading was reported.
    assert not hasUsableHeading(navigation)

    navigation.heading_valid = True
    assert hasUsableHeading(navigation)


def test_usable_heading_rejects_a_degenerate_quaternion():
    navigation = Navigation()
    navigation.heading_valid = True
    navigation.pose.orientation.w = 0.0
    # Flag set but the quaternion is not a rotation: refuse it rather than
    # feeding a garbage yaw downstream.
    assert not hasUsableHeading(navigation)


def test_high_accuracy_gives_the_tighter_sigma(tracker):
    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation(accuracy_high=True))

    expected = 5.0 ** 2
    assert contact.covariance[COV_XX] == pytest.approx(expected)
    assert contact.covariance[COV_YY] == pytest.approx(expected)


def test_low_accuracy_gives_the_looser_sigma(tracker):
    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation(accuracy_high=False))

    expected = 25.0 ** 2
    assert contact.covariance[COV_XX] == pytest.approx(expected)
    assert contact.covariance[COV_YY] == pytest.approx(expected)
    # The whole point of the accuracy bit: a coarse fix must be visibly coarser.
    assert contact.covariance[COV_XX] > 5.0 ** 2


def test_untransmitted_quantities_are_unknown_not_zero(tracker):
    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation())

    # AIS carries no altitude, roll or pitch. Zero variance would assert these
    # are perfectly known, letting a consumer weight garbage infinitely.
    assert contact.covariance[COV_ZZ] == pytest.approx(UNKNOWN)
    assert contact.covariance[COV_RR] == pytest.approx(UNKNOWN)
    assert contact.covariance[COV_PP] == pytest.approx(UNKNOWN)


def test_heading_variance_reflects_whether_a_heading_was_reported(tracker):
    with_heading = AISContact()
    tracker.fillCovariances(with_heading, makeNavigation(heading=True))
    assert with_heading.covariance[COV_AA] == pytest.approx(math.radians(5.0) ** 2)

    without_heading = AISContact()
    tracker.fillCovariances(without_heading, makeNavigation(heading=False))
    assert without_heading.covariance[COV_AA] == pytest.approx(UNKNOWN)


def test_velocity_variance_is_set_when_sog_and_cog_are_present(tracker):
    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation(sog_cog=True))

    expected = 0.5 ** 2
    assert contact.twist.covariance[COV_XX] == pytest.approx(expected)
    assert contact.twist.covariance[COV_YY] == pytest.approx(expected)


def test_missing_sog_cog_marks_velocity_unknown(tracker):
    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation(sog_cog=False))

    # ais_parser writes NaN when the report omitted SOG or COG: the velocity is
    # not merely uncertain, it is absent, and a consumer must be able to tell.
    assert contact.twist.covariance[COV_XX] == pytest.approx(UNKNOWN)
    assert contact.twist.covariance[COV_YY] == pytest.approx(UNKNOWN)


def test_rate_of_turn_variance_follows_its_status(tracker):
    valid = AISContact()
    tracker.fillCovariances(valid, makeNavigation(rate_of_turn_valid=True))
    assert valid.twist.covariance[COV_AA] == pytest.approx(0.02 ** 2)

    unavailable = AISContact()
    tracker.fillCovariances(unavailable, makeNavigation(rate_of_turn_valid=False))
    assert unavailable.twist.covariance[COV_AA] == pytest.approx(UNKNOWN)


def test_no_variance_is_left_at_zero(tracker):
    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation())

    # A zero on the diagonal claims a perfectly known quantity. Nothing AIS
    # reports is perfectly known, so no diagonal entry may be left at zero.
    for index in (COV_XX, COV_YY, COV_ZZ, COV_RR, COV_PP, COV_AA):
        assert contact.covariance[index] > 0.0
        assert contact.twist.covariance[index] > 0.0


def test_sigmas_are_parameters(tracker):
    tracker.set_parameters(
        [rclpy.parameter.Parameter('position_sigma_high',
                                   rclpy.Parameter.Type.DOUBLE, 1.0)])

    contact = AISContact()
    tracker.fillCovariances(contact, makeNavigation(accuracy_high=True))

    # A field config revising the uncertainty model must actually take effect.
    assert contact.covariance[COV_XX] == pytest.approx(1.0)
