// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#include "ais_layer/ais_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "geodesy/ecef.h"
#include "geodesy/wgs84.h"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "marine_ais_msgs/msg/navigational_status.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

PLUGINLIB_EXPORT_CLASS(ais_layer::AISLayer, nav2_costmap_2d::Layer)

namespace ais_layer
{

using marine_ais_msgs::msg::NavigationalStatus;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace
{

/// Distance from a point to a line segment, zero-length segments included.
double pointSegmentDistance(
  const Point2D & point, const Point2D & a, const Point2D & b)
{
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double length_squared = dx * dx + dy * dy;
  if (length_squared <= 0.0) {
    return std::hypot(point.x - a.x, point.y - a.y);
  }
  double t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / length_squared;
  t = std::clamp(t, 0.0, 1.0);
  return std::hypot(point.x - (a.x + t * dx), point.y - (a.y + t * dy));
}

/// Number of vertices of the polygon approximating a heading-less contact.
/// Twelve keeps the inscribed-radius error under 4% while staying cheap.
constexpr int kCircleVertices = 12;

/// Build a regular polygon of @p radius about @p centre.
std::vector<Point2D> circleHull(const Point2D & centre, double radius)
{
  std::vector<Point2D> hull;
  hull.reserve(kCircleVertices);
  for (int i = 0; i < kCircleVertices; ++i) {
    const double angle = 2.0 * M_PI * static_cast<double>(i) / kCircleVertices;
    hull.push_back({centre.x + radius * std::cos(angle),
        centre.y + radius * std::sin(angle)});
  }
  return hull;
}

/// True when a quaternion is numerically usable.
///
/// Deliberately NOT a "heading was reported" test: the ROS default for
/// geometry_msgs/Quaternion is the identity quaternion, so an orientation the
/// parser never touched passes this while carrying no heading at all. Whether
/// a heading exists is answered by the yaw covariance -- see
/// `Navigation.heading_valid`, which the tracker folds into covariance[35].
bool isValidQuaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(norm) && std::abs(norm - 1.0) < 1.0e-3;
}

}  // namespace

bool pointInPolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  // Crossing number. Winding-agnostic on purpose: the hull vertex order comes
  // from the tracker and is not something this layer should depend on.
  bool inside = false;
  const size_t n = polygon.size();
  if (n < 3) {
    return false;
  }
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const Point2D & a = polygon[i];
    const Point2D & b = polygon[j];
    if ((a.y > point.y) != (b.y > point.y)) {
      const double dy = b.y - a.y;
      if (dy != 0.0) {
        const double x_cross = a.x + (point.y - a.y) / dy * (b.x - a.x);
        if (point.x < x_cross) {
          inside = !inside;
        }
      }
    }
  }
  return inside;
}

double distanceToPolygon(const Point2D & point, const std::vector<Point2D> & polygon)
{
  if (polygon.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  if (pointInPolygon(point, polygon)) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  const size_t n = polygon.size();
  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    best = std::min(best, pointSegmentDistance(point, polygon[j], polygon[i]));
  }
  return best;
}

bool AISLayer::isDeadReckonable(uint8_t navigational_status)
{
  // A contact that reports itself held in place is painted where it says it
  // is. Propagating a moored vessel along a stale course is not conservatism,
  // it is fabrication -- and it would smear cost across a harbour.
  switch (navigational_status) {
    case NavigationalStatus::NAVIGATIONAL_STATUS_AT_ANCHOR:
    case NavigationalStatus::NAVIGATIONAL_STATUS_MOORED:
    case NavigationalStatus::NAVIGATIONAL_STATUS_AGROUND:
      return false;
    default:
      return true;
  }
}

double AISLayer::maximumAgeFor(uint8_t position_message_id) const
{
  switch (position_message_id) {
    case 1:
    case 2:
    case 3:
      return max_age_class_a_;
    case 18:
    case 19:
      return max_age_class_b_;
    default:
      // Type 9 (SAR aircraft) and anything else the tracker forwards.
      return max_age_default_;
  }
}

void AISLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("AISLayer: owning node already destroyed");
  }

  current_ = false;
  default_value_ = NO_INFORMATION;
  global_frame_id_ = layered_costmap_->getGlobalFrameID();
  matchSize();

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + ".enabled", enabled_);

  declareParameter("topic", rclcpp::ParameterValue(topic_));
  node->get_parameter(name_ + ".topic", topic_);

  declareParameter("max_age_class_a", rclcpp::ParameterValue(max_age_class_a_));
  node->get_parameter(name_ + ".max_age_class_a", max_age_class_a_);

  declareParameter("max_age_class_b", rclcpp::ParameterValue(max_age_class_b_));
  node->get_parameter(name_ + ".max_age_class_b", max_age_class_b_);

  declareParameter("max_age_default", rclcpp::ParameterValue(max_age_default_));
  node->get_parameter(name_ + ".max_age_default", max_age_default_);

  declareParameter("max_prediction_time", rclcpp::ParameterValue(max_prediction_time_));
  node->get_parameter(name_ + ".max_prediction_time", max_prediction_time_);

  declareParameter("sigma_scale", rclcpp::ParameterValue(sigma_scale_));
  node->get_parameter(name_ + ".sigma_scale", sigma_scale_);

  declareParameter("default_position_sigma", rclcpp::ParameterValue(default_position_sigma_));
  node->get_parameter(name_ + ".default_position_sigma", default_position_sigma_);

  declareParameter("default_speed_sigma", rclcpp::ParameterValue(default_speed_sigma_));
  node->get_parameter(name_ + ".default_speed_sigma", default_speed_sigma_);

  declareParameter("unknown_speed", rclcpp::ParameterValue(unknown_speed_));
  node->get_parameter(name_ + ".unknown_speed", unknown_speed_);

  declareParameter("default_vessel_radius", rclcpp::ParameterValue(default_vessel_radius_));
  node->get_parameter(name_ + ".default_vessel_radius", default_vessel_radius_);

  declareParameter("max_envelope", rclcpp::ParameterValue(max_envelope_));
  node->get_parameter(name_ + ".max_envelope", max_envelope_);

  declareParameter("max_samples", rclcpp::ParameterValue(max_samples_));
  node->get_parameter(name_ + ".max_samples", max_samples_);

  int contact_cost = contact_cost_;
  declareParameter("contact_cost", rclcpp::ParameterValue(contact_cost));
  node->get_parameter(name_ + ".contact_cost", contact_cost);

  int envelope_edge_cost = envelope_edge_cost_;
  declareParameter("envelope_edge_cost", rclcpp::ParameterValue(envelope_edge_cost));
  node->get_parameter(name_ + ".envelope_edge_cost", envelope_edge_cost);

  declareParameter(
    "unknown_variance_threshold", rclcpp::ParameterValue(unknown_variance_threshold_));
  node->get_parameter(name_ + ".unknown_variance_threshold", unknown_variance_threshold_);

  // Parameters are external input, and a field config changes under pressure.
  // Every one of these can silently disable or invert the layer's safety
  // behaviour, so validate rather than trust.
  if (!(std::isfinite(sigma_scale_) && sigma_scale_ > 0.0)) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: sigma_scale must be finite and positive (got "
        << sigma_scale_ << "); using 2.0.");
    sigma_scale_ = 2.0;
  }
  if (!(std::isfinite(max_prediction_time_) && max_prediction_time_ >= 0.0)) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: max_prediction_time must be finite and non-negative (got "
        << max_prediction_time_ << "); using 120.0 s.");
    max_prediction_time_ = 120.0;
  }
  if (!(std::isfinite(max_envelope_) && max_envelope_ > 0.0)) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: max_envelope must be finite and positive (got "
        << max_envelope_ << "); using 500.0 m.");
    max_envelope_ = 500.0;
  }
  if (!(std::isfinite(default_vessel_radius_) && default_vessel_radius_ > 0.0)) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: default_vessel_radius must be finite and positive (got "
        << default_vessel_radius_ << "); using 10.0 m.");
    default_vessel_radius_ = 10.0;
  }
  if (!(std::isfinite(unknown_speed_) && unknown_speed_ >= 0.0)) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: unknown_speed must be finite and non-negative (got "
        << unknown_speed_ << "); using 5.0 m/s.");
    unknown_speed_ = 5.0;
  }
  if (max_samples_ < 1) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: max_samples must be at least 1 (got "
        << max_samples_ << "); using 64.");
    max_samples_ = 64;
  }
  // NO_INFORMATION as a paint value would be read by updateWithMax as "this
  // layer has nothing to say", quietly turning every contact invisible.
  if (contact_cost < 1 || contact_cost >= NO_INFORMATION) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: contact_cost must be in [1, 254] (got "
        << contact_cost << "); using LETHAL_OBSTACLE.");
    contact_cost = LETHAL_OBSTACLE;
  }
  if (envelope_edge_cost < 1 || envelope_edge_cost > contact_cost) {
    RCLCPP_WARN_STREAM(
      logger_, "AISLayer: envelope_edge_cost must be in [1, contact_cost] (got "
        << envelope_edge_cost << "); using 1.");
    envelope_edge_cost = 1;
  }
  contact_cost_ = static_cast<unsigned char>(contact_cost);
  envelope_edge_cost_ = static_cast<unsigned char>(envelope_edge_cost);

  contact_subscriber_ = node->create_subscription<marine_ais_msgs::msg::AISContact>(
    topic_, rclcpp::QoS(50),
    std::bind(&AISLayer::contactCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    logger_,
    "AISLayer subscribed to '%s'; expiry class A %.0f s / class B %.0f s, "
    "prediction capped at %.0f s.",
    topic_.c_str(), max_age_class_a_, max_age_class_b_, max_prediction_time_);
}

void AISLayer::matchSize()
{
  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  resizeMap(
    master->getSizeInCellsX(), master->getSizeInCellsY(),
    master->getResolution(), master->getOriginX(), master->getOriginY());
}

void AISLayer::reset()
{
  resetMaps();
  has_last_bounds_ = false;
  current_ = false;
}

void AISLayer::contactCallback(const marine_ais_msgs::msg::AISContact::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(tracks_mutex_);
  // One live track per MMSI: AIS repeats the whole contact on every report.
  tracks_[msg->id] = *msg;
}

geometry_msgs::msg::Point AISLayer::llToWorld(const geographic_msgs::msg::GeoPoint & geo_point)
{
  geographic_msgs::msg::GeoPoint point = geo_point;
  // AIS carries no altitude. A non-finite value here would propagate straight
  // through the ECEF conversion and poison x and y as well.
  if (!std::isfinite(point.altitude)) {
    point.altitude = 0.0;
  }

  geometry_msgs::msg::PointStamped ecef;
  ecef.header.frame_id = "earth";
  ecef.point = geodesy::toGeometry(geodesy::ECEFPoint(point));

  geometry_msgs::msg::PointStamped world;
  tf_->transform(ecef, world, global_frame_id_);
  return world.point;
}

std::vector<SweptPose> AISLayer::buildSweptPoses(
  const marine_ais_msgs::msg::AISContact & contact,
  const Point2D & origin,
  double age) const
{
  std::vector<SweptPose> poses;

  // --- how well do we know where it was? ---
  double position_sigma = default_position_sigma_;
  const double variance_x = contact.covariance[0];
  const double variance_y = contact.covariance[7];
  if (std::isfinite(variance_x) && std::isfinite(variance_y) &&
    variance_x > 0.0 && variance_y > 0.0 &&
    variance_x < unknown_variance_threshold_ && variance_y < unknown_variance_threshold_)
  {
    position_sigma = std::sqrt(std::max(variance_x, variance_y));
  }

  // --- how well do we know where it is going? ---
  double velocity_x = contact.twist.twist.linear.x;
  double velocity_y = contact.twist.twist.linear.y;
  const bool velocity_known = std::isfinite(velocity_x) && std::isfinite(velocity_y);
  const bool movable = isDeadReckonable(contact.navigational_status.status);

  double speed_sigma = default_speed_sigma_;
  const double speed_variance = contact.twist.covariance[0];
  if (std::isfinite(speed_variance) && speed_variance > 0.0 &&
    speed_variance < unknown_variance_threshold_)
  {
    speed_sigma = std::sqrt(speed_variance);
  }

  const bool dead_reckon = velocity_known && movable;
  if (!dead_reckon) {
    velocity_x = 0.0;
    velocity_y = 0.0;
  }

  // Rate at which the uncertainty envelope widens, metres per second.
  //   - dead-reckoning: the velocity estimate's own error
  //   - movable but velocity unknown: the whole reachable set opens up
  //   - reporting itself stationary: it does not grow at all
  double growth_rate = 0.0;
  if (dead_reckon) {
    growth_rate = speed_sigma;
  } else if (movable) {
    growth_rate = unknown_speed_;
  }

  double prediction_time = age;
  if (!std::isfinite(prediction_time) || prediction_time < 0.0) {
    prediction_time = 0.0;
  }
  prediction_time = std::min(prediction_time, max_prediction_time_);

  // --- outline ---
  std::vector<Point2D> body_hull;
  double circumscribed_radius = 0.0;
  for (const auto & p : contact.footprint.points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
      body_hull.clear();
      break;
    }
    const double px = static_cast<double>(p.x);
    const double py = static_cast<double>(p.y);
    body_hull.push_back({px, py});
    circumscribed_radius = std::max(circumscribed_radius, std::hypot(px, py));
  }
  // calculatePolygon closes the ring by repeating the bow; the crossing-number
  // test treats the vertex list as implicitly closed, so drop the duplicate.
  if (body_hull.size() > 1 &&
    std::abs(body_hull.front().x - body_hull.back().x) < 1.0e-9 &&
    std::abs(body_hull.front().y - body_hull.back().y) < 1.0e-9)
  {
    body_hull.pop_back();
  }

  // Whether a true heading was actually reported is carried in the yaw
  // covariance, not in the quaternion: an unreported heading leaves the
  // identity quaternion behind, which reads as a perfectly good "due east".
  // The tracker writes the unknown-variance stand-in when no heading came in.
  double yaw = 0.0;
  bool yaw_known = false;
  const double heading_variance = contact.covariance[35];
  const bool heading_reported =
    std::isfinite(heading_variance) && heading_variance > 0.0 &&
    heading_variance < unknown_variance_threshold_;
  if (heading_reported && isValidQuaternion(contact.pose.orientation)) {
    yaw = tf2::getYaw(contact.pose.orientation);
    yaw_known = std::isfinite(yaw);
  }
  if (!yaw_known && dead_reckon) {
    const double speed = std::hypot(velocity_x, velocity_y);
    // Below a knot or so, course-over-ground is noise rather than heading.
    if (speed > 0.5) {
      yaw = std::atan2(velocity_y, velocity_x);
      yaw_known = true;
    }
  }

  // Without a heading, an oriented hull would be a fabrication. Fall back to a
  // circle that still covers the vessel's true extent.
  const bool use_circle = body_hull.size() < 3 || !yaw_known;
  const double circle_radius =
    (circumscribed_radius > 0.0) ? circumscribed_radius : default_vessel_radius_;

  // --- sample the corridor ---
  const double speed = std::hypot(velocity_x, velocity_y);
  const double travel = speed * prediction_time;
  double resolution = getResolution();
  if (!(std::isfinite(resolution) && resolution > 0.0)) {
    resolution = 1.0;
  }

  int samples = 0;
  if (travel > 0.0) {
    samples = static_cast<int>(std::ceil(travel / resolution));
    samples = std::clamp(samples, 1, max_samples_);
  }

  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);

  const auto emit = [&](double t) {
      SweptPose pose;
      pose.envelope = std::min(
        sigma_scale_ * (position_sigma + growth_rate * t), max_envelope_);
      const Point2D centre{origin.x + velocity_x * t, origin.y + velocity_y * t};
      if (use_circle) {
        pose.hull = circleHull(centre, circle_radius);
      } else {
        pose.hull.reserve(body_hull.size());
        for (const auto & p : body_hull) {
          pose.hull.push_back(
            {centre.x + p.x * cos_yaw - p.y * sin_yaw,
              centre.y + p.x * sin_yaw + p.y * cos_yaw});
        }
      }
      poses.push_back(pose);
    };

  if (samples == 0) {
    // Stationary, or moving-but-unknown: one outline at the fix. The envelope
    // is taken at the full prediction time, which is the worst case and the
    // only honest one.
    emit(prediction_time);
  } else {
    for (int i = 0; i <= samples; ++i) {
      emit(prediction_time * static_cast<double>(i) / samples);
    }
  }

  return poses;
}

void AISLayer::paintSweptPose(
  const SweptPose & pose, double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (pose.hull.size() < 3) {
    return;
  }

  double hull_min_x = std::numeric_limits<double>::max();
  double hull_min_y = std::numeric_limits<double>::max();
  double hull_max_x = std::numeric_limits<double>::lowest();
  double hull_max_y = std::numeric_limits<double>::lowest();
  for (const auto & p : pose.hull) {
    hull_min_x = std::min(hull_min_x, p.x);
    hull_min_y = std::min(hull_min_y, p.y);
    hull_max_x = std::max(hull_max_x, p.x);
    hull_max_y = std::max(hull_max_y, p.y);
  }

  const double envelope = std::max(pose.envelope, 0.0);
  hull_min_x -= envelope;
  hull_min_y -= envelope;
  hull_max_x += envelope;
  hull_max_y += envelope;

  int cell_min_x, cell_min_y, cell_max_x, cell_max_y;
  worldToMapEnforceBounds(hull_min_x, hull_min_y, cell_min_x, cell_min_y);
  worldToMapEnforceBounds(hull_max_x, hull_max_y, cell_max_x, cell_max_y);

  const double cost_span =
    static_cast<double>(contact_cost_) - static_cast<double>(envelope_edge_cost_);

  for (int j = cell_min_y; j <= cell_max_y; ++j) {
    for (int i = cell_min_x; i <= cell_max_x; ++i) {
      double world_x, world_y;
      mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), world_x, world_y);
      const double distance = distanceToPolygon({world_x, world_y}, pose.hull);

      unsigned char cost;
      if (distance <= 0.0) {
        cost = contact_cost_;
      } else if (envelope > 0.0 && distance < envelope) {
        // Linear taper from the hull edge out to the envelope edge. The point
        // is that the planner can see the gradient and prefer the outside of
        // it, rather than facing a cliff at an arbitrary radius.
        const double ramp = 1.0 - distance / envelope;
        cost = static_cast<unsigned char>(
          std::lround(static_cast<double>(envelope_edge_cost_) + cost_span * ramp));
      } else {
        continue;
      }

      const unsigned int index = getIndex(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j));
      // Max-combine within our own layer: swept poses overlap heavily, and a
      // later, wider, softer pose must not erase an earlier lethal hull.
      if (costmap_[index] == NO_INFORMATION || cost > costmap_[index]) {
        costmap_[index] = cost;
      }
      touch(world_x, world_y, min_x, min_y, max_x, max_y);
    }
  }
}

void AISLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }

  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
  }

  // Every contact has moved since last cycle, so last cycle's paint is stale by
  // construction. Clear it, and keep asking for its extent so the master clears
  // there too -- nav2 only resets the master over the bounds layers request.
  resetMaps();
  if (has_last_bounds_) {
    *min_x = std::min(*min_x, last_min_x_);
    *min_y = std::min(*min_y, last_min_y_);
    *max_x = std::max(*max_x, last_max_x_);
    *max_y = std::max(*max_y, last_max_y_);
  }

  std::map<uint32_t, marine_ais_msgs::msg::AISContact> tracks;
  {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    tracks = tracks_;
  }

  const rclcpp::Time now = clock_->now();

  double painted_min_x = std::numeric_limits<double>::max();
  double painted_min_y = std::numeric_limits<double>::max();
  double painted_max_x = std::numeric_limits<double>::lowest();
  double painted_max_y = std::numeric_limits<double>::lowest();
  bool painted = false;

  std::vector<uint32_t> expired;

  for (const auto & entry : tracks) {
    const marine_ais_msgs::msg::AISContact & contact = entry.second;

    const rclcpp::Time stamp(contact.header.stamp);
    if (stamp.nanoseconds() == 0) {
      // Unstamped contact: age is unknowable, so the uncertainty model has no
      // input. Treating it as fresh would understate the envelope, which is the
      // dangerous direction, so drop it and say so.
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "AISLayer: contact %u has no timestamp; dropping.", entry.first);
      expired.push_back(entry.first);
      continue;
    }

    double age = (now - stamp).seconds();
    if (age < 0.0) {
      // Clock skew between the receiver host and this one. Not fatal, but the
      // prediction has nothing to extrapolate over.
      age = 0.0;
    }

    if (age > maximumAgeFor(contact.position_message_id)) {
      expired.push_back(entry.first);
      continue;
    }

    // ais_parser writes NaN into latitude/longitude when the report omitted
    // them. Converting that would yield a NaN position that silently paints
    // nothing, or worse, an enormous bounding box.
    if (!std::isfinite(contact.pose.position.latitude) ||
      !std::isfinite(contact.pose.position.longitude))
    {
      continue;
    }

    Point2D origin;
    try {
      const geometry_msgs::msg::Point world = llToWorld(contact.pose.position);
      if (!std::isfinite(world.x) || !std::isfinite(world.y)) {
        continue;
      }
      origin = {world.x, world.y};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 10000,
        "AISLayer: cannot transform contact %u into %s: %s",
        entry.first, global_frame_id_.c_str(), e.what());
      continue;
    }

    for (const auto & pose : buildSweptPoses(contact, origin, age)) {
      paintSweptPose(pose, &painted_min_x, &painted_min_y, &painted_max_x, &painted_max_y);
      painted = true;
    }
  }

  if (!expired.empty()) {
    std::lock_guard<std::mutex> lock(tracks_mutex_);
    for (const uint32_t id : expired) {
      tracks_.erase(id);
    }
  }

  if (painted && painted_max_x >= painted_min_x) {
    *min_x = std::min(*min_x, painted_min_x);
    *min_y = std::min(*min_y, painted_min_y);
    *max_x = std::max(*max_x, painted_max_x);
    *max_y = std::max(*max_y, painted_max_y);
    last_min_x_ = painted_min_x;
    last_min_y_ = painted_min_y;
    last_max_x_ = painted_max_x;
    last_max_y_ = painted_max_y;
    has_last_bounds_ = true;
  } else {
    // No contacts in range: the layer contributes nothing this cycle. This is
    // also what a dead feed looks like once everything ages out, and it must
    // stay a no-op rather than an obstruction.
    has_last_bounds_ = false;
  }

  current_ = true;
}

void AISLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  updateWithMax(master_grid, min_i, min_j, max_i, max_j);
}

}  // namespace ais_layer
