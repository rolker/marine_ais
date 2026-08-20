// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "ais_layer/ais_layer.hpp"
#include "marine_ais_msgs/msg/ais_contact.hpp"
#include "marine_ais_msgs/msg/navigational_status.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

using ais_layer::Point2D;
using ais_layer::SweptPose;
using ais_layer::Hull;
using ais_layer::distanceToHull;
using ais_layer::distanceToPolygon;
using ais_layer::makeHull;
using ais_layer::pointInPolygon;
using marine_ais_msgs::msg::AISContact;
using marine_ais_msgs::msg::NavigationalStatus;

namespace
{

/// Exposes the layer's tunables so tests can exercise the model without
/// standing up a lifecycle node to feed it parameters.
class TestableAISLayer : public ais_layer::AISLayer
{
public:
  using AISLayer::default_position_sigma_;
  using AISLayer::default_speed_sigma_;
  using AISLayer::default_vessel_radius_;
  using AISLayer::max_age_class_a_;
  using AISLayer::max_age_class_b_;
  using AISLayer::max_age_default_;
  using AISLayer::max_envelope_;
  using AISLayer::max_prediction_time_;
  using AISLayer::max_samples_;
  using AISLayer::sigma_scale_;
  using AISLayer::unknown_speed_;
  // Clearing internals, for the NO_INFORMATION regression test below.
  using AISLayer::clearLastPaint;
  using AISLayer::has_last_bounds_;
  using AISLayer::last_max_x_;
  using AISLayer::last_max_y_;
  using AISLayer::last_min_x_;
  using AISLayer::last_min_y_;
  using nav2_costmap_2d::Costmap2D::default_value_;
  using nav2_costmap_2d::Costmap2D::resetMaps;
};

/// A symmetric rectangular hull, so the vertex mean equals the placed centre
/// and tests can recover the position a pose was placed at.
std::vector<geometry_msgs::msg::Point32> rectangleFootprint(float half_length, float half_width)
{
  std::vector<geometry_msgs::msg::Point32> points;
  const float xs[4] = {half_length, -half_length, -half_length, half_length};
  const float ys[4] = {half_width, half_width, -half_width, -half_width};
  for (int i = 0; i < 4; ++i) {
    geometry_msgs::msg::Point32 p;
    p.x = xs[i];
    p.y = ys[i];
    points.push_back(p);
  }
  return points;
}

AISContact makeContact(
  double velocity_x, double velocity_y, uint8_t status,
  double position_sigma = 5.0, double speed_sigma = 0.5)
{
  AISContact contact;
  contact.id = 366123456;
  contact.position_message_id = 1;
  contact.navigational_status.status = status;

  contact.twist.twist.linear.x = velocity_x;
  contact.twist.twist.linear.y = velocity_y;

  contact.covariance[0] = position_sigma * position_sigma;
  contact.covariance[7] = position_sigma * position_sigma;
  contact.twist.covariance[0] = speed_sigma * speed_sigma;

  // Identity quaternion: a valid rotation with yaw 0 (bow due east in ENU).
  // The yaw covariance is what says a heading was actually REPORTED -- the
  // quaternion alone cannot, since its ROS default is already the identity.
  contact.pose.orientation.w = 1.0;
  contact.covariance[35] = 0.0076;  // (5 deg)^2, i.e. heading present

  contact.footprint.points = rectangleFootprint(5.0f, 3.0f);
  return contact;
}

const std::vector<Point2D> kUnitSquare = {{0.0, 0.0}, {10.0, 0.0}, {10.0, 10.0}, {0.0, 10.0}};

}  // namespace

// --- geometry ------------------------------------------------------------

TEST(Geometry, PointInPolygonInsideAndOutside)
{
  EXPECT_TRUE(pointInPolygon({5.0, 5.0}, kUnitSquare));
  EXPECT_FALSE(pointInPolygon({-1.0, 5.0}, kUnitSquare));
  EXPECT_FALSE(pointInPolygon({11.0, 5.0}, kUnitSquare));
  EXPECT_FALSE(pointInPolygon({5.0, -0.5}, kUnitSquare));
}

TEST(Geometry, PointInPolygonRejectsDegenerate)
{
  EXPECT_FALSE(pointInPolygon({0.0, 0.0}, {}));
  EXPECT_FALSE(pointInPolygon({0.0, 0.0}, {{0.0, 0.0}, {1.0, 1.0}}));
}

TEST(Geometry, DistanceToPolygonIsZeroInside)
{
  EXPECT_DOUBLE_EQ(0.0, distanceToPolygon({5.0, 5.0}, kUnitSquare));
}

TEST(Geometry, DistanceToPolygonOutside)
{
  EXPECT_NEAR(2.0, distanceToPolygon({12.0, 5.0}, kUnitSquare), 1e-9);
  EXPECT_NEAR(3.0, distanceToPolygon({5.0, -3.0}, kUnitSquare), 1e-9);
  // Diagonal from a corner.
  EXPECT_NEAR(std::hypot(3.0, 4.0), distanceToPolygon({13.0, 14.0}, kUnitSquare), 1e-9);
}

TEST(Geometry, DistanceToEmptyPolygonIsInfinite)
{
  EXPECT_TRUE(std::isinf(distanceToPolygon({0.0, 0.0}, {})));
}

// --- status gating -------------------------------------------------------

TEST(StatusGating, HeldVesselsAreNotDeadReckoned)
{
  EXPECT_FALSE(
    TestableAISLayer::isDeadReckonable(NavigationalStatus::NAVIGATIONAL_STATUS_AT_ANCHOR));
  EXPECT_FALSE(
    TestableAISLayer::isDeadReckonable(NavigationalStatus::NAVIGATIONAL_STATUS_MOORED));
  EXPECT_FALSE(
    TestableAISLayer::isDeadReckonable(NavigationalStatus::NAVIGATIONAL_STATUS_AGROUND));
}

TEST(StatusGating, MovingVesselsAreDeadReckoned)
{
  EXPECT_TRUE(
    TestableAISLayer::isDeadReckonable(NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE));
  EXPECT_TRUE(
    TestableAISLayer::isDeadReckonable(NavigationalStatus::NAVIGATIONAL_STATUS_ENGAGED_IN_FISHING));
  // An undefined status is the common case for a contact whose report omitted
  // it. Treat it as movable: assuming stationary is the dangerous direction.
  EXPECT_TRUE(
    TestableAISLayer::isDeadReckonable(NavigationalStatus::NAVIGATIONAL_STATUS_UNDEFINED));
}

// --- per-class expiry ----------------------------------------------------

TEST(Expiry, IsPerReportingClass)
{
  TestableAISLayer layer;
  layer.max_age_class_a_ = 90.0;
  layer.max_age_class_b_ = 180.0;
  layer.max_age_default_ = 120.0;

  // Class A position reports: types 1, 2, 3.
  EXPECT_DOUBLE_EQ(90.0, layer.maximumAgeFor(1));
  EXPECT_DOUBLE_EQ(90.0, layer.maximumAgeFor(2));
  EXPECT_DOUBLE_EQ(90.0, layer.maximumAgeFor(3));
  // Class B: types 18, 19 -- reports far less often, so it must live longer.
  EXPECT_DOUBLE_EQ(180.0, layer.maximumAgeFor(18));
  EXPECT_DOUBLE_EQ(180.0, layer.maximumAgeFor(19));
  // Type 9 (SAR aircraft) and anything else.
  EXPECT_DOUBLE_EQ(120.0, layer.maximumAgeFor(9));
}

// --- the interpolation model --------------------------------------------

TEST(SweptPoses, StationaryContactDoesNotMove)
{
  TestableAISLayer layer;
  const auto contact =
    makeContact(4.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);

  // Velocity is present but the vessel says it is moored: propagating it would
  // smear cost across the harbour on a stale course.
  const auto poses = layer.buildSweptPoses(contact, {100.0, 200.0}, 60.0);

  ASSERT_EQ(1u, poses.size());
  const Point2D placed = poses.front().hull.centre;
  EXPECT_NEAR(100.0, placed.x, 1e-6);
  EXPECT_NEAR(200.0, placed.y, 1e-6);
}

TEST(SweptPoses, StationaryContactEnvelopeDoesNotGrow)
{
  TestableAISLayer layer;
  layer.sigma_scale_ = 2.0;
  const auto contact = makeContact(
    0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_AT_ANCHOR, 5.0);

  const auto fresh = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);
  const auto stale = layer.buildSweptPoses(contact, {0.0, 0.0}, 100.0);

  ASSERT_EQ(1u, fresh.size());
  ASSERT_EQ(1u, stale.size());
  // 2.0 * 5.0 m, regardless of age: an anchored vessel's position uncertainty
  // is the AIS fix error and nothing else.
  EXPECT_NEAR(10.0, fresh.front().envelope, 1e-6);
  EXPECT_NEAR(10.0, stale.front().envelope, 1e-6);
}

TEST(SweptPoses, MovingContactSweepsFromFixToPrediction)
{
  TestableAISLayer layer;
  const auto contact = makeContact(
    5.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE);

  // 5 m/s east for 20 s = 100 m of travel.
  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 20.0);

  ASSERT_GE(poses.size(), 2u);
  const Point2D first = poses.front().hull.centre;
  const Point2D last = poses.back().hull.centre;
  EXPECT_NEAR(0.0, first.x, 1e-6);
  EXPECT_NEAR(0.0, first.y, 1e-6);
  EXPECT_NEAR(100.0, last.x, 1e-6);
  EXPECT_NEAR(0.0, last.y, 1e-6);
}

TEST(SweptPoses, EnvelopeGrowsMonotonicallyAlongTheCorridor)
{
  TestableAISLayer layer;
  const auto contact = makeContact(
    5.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE);

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 30.0);

  ASSERT_GE(poses.size(), 2u);
  for (size_t i = 1; i < poses.size(); ++i) {
    EXPECT_GE(poses[i].envelope, poses[i - 1].envelope)
      << "envelope shrank between sample " << i - 1 << " and " << i;
  }
  // The whole point: the far end is less certain than the near end.
  EXPECT_GT(poses.back().envelope, poses.front().envelope);
}

TEST(SweptPoses, UnknownVelocityGrowsAReachableSetWithoutMoving)
{
  TestableAISLayer layer;
  layer.sigma_scale_ = 1.0;
  layer.unknown_speed_ = 5.0;

  auto contact = makeContact(
    0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE, 10.0);
  // ais_parser writes NaN when the report carried no SOG/COG.
  contact.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  contact.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 20.0);

  // Nowhere to propagate to, but the vessel could be anywhere within reach.
  ASSERT_EQ(1u, poses.size());
  const Point2D placed = poses.front().hull.centre;
  EXPECT_NEAR(0.0, placed.x, 1e-6);
  EXPECT_NEAR(0.0, placed.y, 1e-6);
  // 1.0 * (10 m fix error + 5 m/s * 20 s) = 110 m.
  EXPECT_NEAR(110.0, poses.front().envelope, 1e-6);
}

TEST(SweptPoses, PredictionIsCappedSoAncientContactsDoNotFlyOffTheMap)
{
  TestableAISLayer layer;
  layer.max_prediction_time_ = 30.0;
  const auto contact = makeContact(
    10.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE);

  // Ten minutes stale, but prediction stops at 30 s => 300 m, not 6000 m.
  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 600.0);

  ASSERT_GE(poses.size(), 2u);
  EXPECT_NEAR(300.0, poses.back().hull.centre.x, 1e-6);
}

TEST(SweptPoses, EnvelopeIsCapped)
{
  TestableAISLayer layer;
  layer.max_envelope_ = 50.0;
  layer.sigma_scale_ = 1.0;
  layer.unknown_speed_ = 100.0;

  auto contact = makeContact(
    0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE);
  contact.twist.twist.linear.x = std::numeric_limits<double>::quiet_NaN();
  contact.twist.twist.linear.y = std::numeric_limits<double>::quiet_NaN();

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 600.0);

  ASSERT_FALSE(poses.empty());
  for (const auto & pose : poses) {
    EXPECT_LE(pose.envelope, 50.0);
  }
}

TEST(SweptPoses, SampleCountIsBounded)
{
  TestableAISLayer layer;
  layer.max_samples_ = 8;
  layer.max_prediction_time_ = 1000.0;
  const auto contact = makeContact(
    20.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE);

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 1000.0);

  // samples + 1 poses, and samples is clamped.
  EXPECT_LE(poses.size(), 9u);
}

TEST(SweptPoses, PositionCovarianceDrivesTheEnvelope)
{
  TestableAISLayer layer;
  layer.sigma_scale_ = 2.0;

  // The tracker's covariance fix is what makes this distinction possible: a
  // low-accuracy AIS fix must produce a visibly bigger envelope.
  const auto accurate =
    makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED, 5.0);
  const auto coarse =
    makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED, 25.0);

  const auto accurate_poses = layer.buildSweptPoses(accurate, {0.0, 0.0}, 0.0);
  const auto coarse_poses = layer.buildSweptPoses(coarse, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(accurate_poses.empty());
  ASSERT_FALSE(coarse_poses.empty());
  EXPECT_NEAR(10.0, accurate_poses.front().envelope, 1e-6);
  EXPECT_NEAR(50.0, coarse_poses.front().envelope, 1e-6);
}

TEST(SweptPoses, UnpopulatedCovarianceFallsBackToTheDefault)
{
  TestableAISLayer layer;
  layer.sigma_scale_ = 1.0;
  layer.default_position_sigma_ = 25.0;

  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  // A tracker predating the covariance fix leaves these zero-filled. Zero
  // variance would claim a perfectly known position.
  contact.covariance[0] = 0.0;
  contact.covariance[7] = 0.0;

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(25.0, poses.front().envelope, 1e-6);
}

TEST(SweptPoses, UnknownVarianceSentinelFallsBackToTheDefault)
{
  TestableAISLayer layer;
  layer.sigma_scale_ = 1.0;
  layer.default_position_sigma_ = 25.0;

  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  // The tracker's "unknown" stand-in must not be read as a 1000 m sigma.
  contact.covariance[0] = 1.0e6;
  contact.covariance[7] = 1.0e6;

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  EXPECT_NEAR(25.0, poses.front().envelope, 1e-6);
}

TEST(SweptPoses, KnownHeadingOrientsTheHull)
{
  TestableAISLayer layer;
  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  // Identity quaternion => yaw 0 => bow along +x. Hull is 10 m by 6 m.
  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  double max_x = 0.0;
  double max_y = 0.0;
  for (const auto & p : poses.front().hull.vertices) {
    max_x = std::max(max_x, std::abs(p.x));
    max_y = std::max(max_y, std::abs(p.y));
  }
  EXPECT_NEAR(5.0, max_x, 1e-6);
  EXPECT_NEAR(3.0, max_y, 1e-6);
}

TEST(SweptPoses, MissingHeadingFallsBackToACircle)
{
  TestableAISLayer layer;
  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  // The trap: ais_parser leaves the IDENTITY quaternion when no heading was
  // reported, which is a perfectly valid rotation meaning due east. Only the
  // unknown yaw variance distinguishes "no heading" from "heading 090".
  contact.pose.orientation.w = 1.0;
  contact.covariance[35] = 1.0e6;

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  EXPECT_TRUE(poses.front().hull.is_circle);
  EXPECT_TRUE(poses.front().hull.vertices.empty());
  EXPECT_NEAR(std::hypot(5.0, 3.0), poses.front().hull.radius, 1e-6);
}

TEST(SweptPoses, MissingFootprintFallsBackToTheDefaultRadius)
{
  TestableAISLayer layer;
  layer.default_vessel_radius_ = 12.0;
  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  // No static report received yet, so no hull dimensions.
  contact.footprint.points.clear();

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  EXPECT_TRUE(poses.front().hull.is_circle);
  EXPECT_NEAR(12.0, poses.front().hull.radius, 1e-6);
}

TEST(SweptPoses, NonFiniteFootprintIsRejected)
{
  TestableAISLayer layer;
  layer.default_vessel_radius_ = 12.0;
  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  contact.footprint.points[1].x = std::numeric_limits<float>::quiet_NaN();

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  // A NaN vertex would make every distance comparison false and silently
  // punch a hole in the painted hull; fall back to the circle instead.
  ASSERT_FALSE(poses.empty());
  EXPECT_TRUE(poses.front().hull.is_circle);
  EXPECT_TRUE(std::isfinite(poses.front().hull.radius));
  EXPECT_GT(poses.front().hull.radius, 0.0);
}

TEST(SweptPoses, NegativeAgeIsClamped)
{
  TestableAISLayer layer;
  layer.sigma_scale_ = 1.0;
  const auto contact = makeContact(
    5.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE, 5.0);

  // Clock skew between the receiver host and this one.
  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, -30.0);

  ASSERT_EQ(1u, poses.size());
  EXPECT_NEAR(0.0, poses.front().hull.centre.x, 1e-6);
  EXPECT_NEAR(5.0, poses.front().envelope, 1e-6);
}

TEST(SweptPoses, CourseProvidesHeadingWhenTrueHeadingIsAbsent)
{
  TestableAISLayer layer;
  auto contact = makeContact(
    0.0, 5.0, NavigationalStatus::NAVIGATIONAL_STATUS_UNDER_WAY_ENGINE);
  contact.covariance[35] = 1.0e6;  // no true heading reported

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  // Making way due north, so the 10 m hull should be long in y, not x.
  double max_x = 0.0;
  double max_y = 0.0;
  for (const auto & p : poses.front().hull.vertices) {
    max_x = std::max(max_x, std::abs(p.x));
    max_y = std::max(max_y, std::abs(p.y));
  }
  EXPECT_NEAR(3.0, max_x, 1e-6);
  EXPECT_NEAR(5.0, max_y, 1e-6);
}

TEST(SweptPoses, IdentityQuaternionIsNotMistakenForAHeading)
{
  TestableAISLayer layer;
  auto contact = makeContact(0.0, 0.0, NavigationalStatus::NAVIGATIONAL_STATUS_MOORED);
  // Exactly what a report with no true heading and no way on produces: an
  // untouched (identity) quaternion. Orienting a 10 m hull due east off the
  // back of that would be a fabrication, so the layer must fall back to a
  // circle rather than trust it.
  contact.pose.orientation.x = 0.0;
  contact.pose.orientation.y = 0.0;
  contact.pose.orientation.z = 0.0;
  contact.pose.orientation.w = 1.0;
  contact.covariance[35] = 1.0e6;

  const auto poses = layer.buildSweptPoses(contact, {0.0, 0.0}, 0.0);

  ASSERT_FALSE(poses.empty());
  EXPECT_TRUE(poses.front().hull.is_circle);
  EXPECT_NEAR(std::hypot(5.0, 3.0), poses.front().hull.radius, 1e-6);
}

// --- hull construction and the fast distance paths ---------------------

TEST(HullGeometry, NoHeadingGivesACircleCoveringTheVesselExtent)
{
  const std::vector<Point2D> body = {{5.0, 3.0}, {-5.0, 3.0}, {-5.0, -3.0}, {5.0, -3.0}};
  const Hull hull = makeHull({10.0, 20.0}, 0.0, false, body, 7.0);

  EXPECT_TRUE(hull.is_circle);
  EXPECT_NEAR(10.0, hull.centre.x, 1e-9);
  EXPECT_NEAR(20.0, hull.centre.y, 1e-9);
  // Circumscribed, so the circle still covers the corners of the real hull.
  EXPECT_NEAR(std::hypot(5.0, 3.0), hull.radius, 1e-9);
}

TEST(HullGeometry, NoDimensionsFallsBackToTheGivenRadius)
{
  const Hull hull = makeHull({0.0, 0.0}, 0.0, true, {}, 7.0);
  EXPECT_TRUE(hull.is_circle);
  EXPECT_NEAR(7.0, hull.radius, 1e-9);
}

TEST(HullGeometry, OrientedHullCarriesUsableBoundingCircles)
{
  const std::vector<Point2D> body = {{5.0, 3.0}, {-5.0, 3.0}, {-5.0, -3.0}, {5.0, -3.0}};
  const Hull hull = makeHull({0.0, 0.0}, 0.0, true, body, 7.0);

  ASSERT_FALSE(hull.is_circle);
  ASSERT_EQ(4u, hull.vertices.size());
  // The rasteriser rejects on the circumscribed circle and accepts on the
  // inscribed one, so both must bound the real outline correctly.
  EXPECT_NEAR(std::hypot(5.0, 3.0), hull.radius, 1e-9);
  EXPECT_NEAR(3.0, hull.inscribed, 1e-9);
  for (const auto & v : hull.vertices) {
    EXPECT_LE(std::hypot(v.x, v.y), hull.radius + 1e-9);
    EXPECT_GE(std::hypot(v.x, v.y), hull.inscribed - 1e-9);
  }
}

TEST(HullGeometry, CircleDistanceIsAnalytic)
{
  const Hull hull = makeHull({0.0, 0.0}, 0.0, false, {}, 10.0);
  EXPECT_DOUBLE_EQ(0.0, distanceToHull({0.0, 0.0}, hull));
  EXPECT_DOUBLE_EQ(0.0, distanceToHull({10.0, 0.0}, hull));
  EXPECT_NEAR(5.0, distanceToHull({15.0, 0.0}, hull), 1e-9);
  EXPECT_NEAR(2.0, distanceToHull({0.0, -12.0}, hull), 1e-9);
}

TEST(HullGeometry, PolygonFastPathAgreesWithTheExactDistance)
{
  const std::vector<Point2D> body = {{5.0, 3.0}, {-5.0, 3.0}, {-5.0, -3.0}, {5.0, -3.0}};
  const Hull hull = makeHull({0.0, 0.0}, 0.0, true, body, 7.0);

  // The inscribed-circle shortcut must never disagree with the real polygon
  // distance -- that shortcut is the whole performance fix, and a wrong
  // answer inside the hull would silently stop painting a vessel.
  for (double x = -12.0; x <= 12.0; x += 0.5) {
    for (double y = -12.0; y <= 12.0; y += 0.5) {
      const Point2D p{x, y};
      EXPECT_NEAR(distanceToPolygon(p, hull.vertices), distanceToHull(p, hull), 1e-9)
        << "disagreement at (" << x << ", " << y << ")";
    }
  }
}

TEST(HullGeometry, RotatedHullPlacesVerticesCorrectly)
{
  const std::vector<Point2D> body = {{5.0, 3.0}, {-5.0, 3.0}, {-5.0, -3.0}, {5.0, -3.0}};
  // 90 degrees: bow swings from +x to +y.
  const Hull hull = makeHull({0.0, 0.0}, M_PI / 2.0, true, body, 7.0);

  ASSERT_FALSE(hull.is_circle);
  double max_x = 0.0, max_y = 0.0;
  for (const auto & v : hull.vertices) {
    max_x = std::max(max_x, std::abs(v.x));
    max_y = std::max(max_y, std::abs(v.y));
  }
  EXPECT_NEAR(3.0, max_x, 1e-9);
  EXPECT_NEAR(5.0, max_y, 1e-9);
}

TEST(ClearPaint, RestoresNoInformationNeverFreeSpace)
{
  TestableAISLayer layer;
  layer.default_value_ = nav2_costmap_2d::NO_INFORMATION;
  layer.resizeMap(20, 20, 1.0, 0.0, 0.0);
  layer.resetMaps();
  ASSERT_EQ(nav2_costmap_2d::NO_INFORMATION, layer.getCost(5, 5));

  layer.setCost(5, 5, nav2_costmap_2d::LETHAL_OBSTACLE);
  layer.setCost(6, 6, 200);
  layer.last_min_x_ = 4.0;
  layer.last_min_y_ = 4.0;
  layer.last_max_x_ = 7.0;
  layer.last_max_y_ = 7.0;
  layer.has_last_bounds_ = true;

  layer.clearLastPaint();

  // Must come back UNKNOWN. FREE_SPACE here would make updateWithMax overwrite
  // unknown master cells with free -- i.e. this layer declaring unsurveyed
  // water navigable, which is the worst thing an obstacle layer can do.
  EXPECT_EQ(nav2_costmap_2d::NO_INFORMATION, layer.getCost(5, 5));
  EXPECT_EQ(nav2_costmap_2d::NO_INFORMATION, layer.getCost(6, 6));
  EXPECT_NE(nav2_costmap_2d::FREE_SPACE, layer.getCost(5, 5));
}

TEST(ClearPaint, LeavesCellsOutsideTheLastBoundsAlone)
{
  TestableAISLayer layer;
  layer.default_value_ = nav2_costmap_2d::NO_INFORMATION;
  layer.resizeMap(20, 20, 1.0, 0.0, 0.0);
  layer.resetMaps();

  layer.setCost(2, 2, 111);   // outside the window below
  layer.setCost(5, 5, 222);   // inside
  layer.last_min_x_ = 4.0;
  layer.last_min_y_ = 4.0;
  layer.last_max_x_ = 7.0;
  layer.last_max_y_ = 7.0;
  layer.has_last_bounds_ = true;

  layer.clearLastPaint();

  // The whole point of the bounded clear: it must not touch the rest of a
  // 16-million-cell map.
  EXPECT_EQ(111, layer.getCost(2, 2));
  EXPECT_EQ(nav2_costmap_2d::NO_INFORMATION, layer.getCost(5, 5));
}
