// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

#ifndef AIS_LAYER__AIS_LAYER_HPP_
#define AIS_LAYER__AIS_LAYER_HPP_

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include "geographic_msgs/msg/geo_point.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "marine_ais_msgs/msg/ais_contact.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ais_layer
{

/// A single vertex in the costmap's world frame (metres).
struct Point2D
{
  double x{0.0};
  double y{0.0};
};

/// @brief A vessel outline placed at one point along a dead-reckoned track,
///        together with the uncertainty envelope that applies at that instant.
///
/// The layer paints a sequence of these: one at the last received fix, one at
/// the predicted present position, and a run of them in between. `envelope`
/// grows monotonically along that sequence, which is what makes the painted
/// region a widening corridor rather than a constant-width sausage.
struct SweptPose
{
  std::vector<Point2D> hull;   ///< Vessel outline in world coordinates.
  double envelope{0.0};        ///< Uncertainty margin outside the hull, metres.
};

/// @brief Nav2 costmap layer fed by AIS contacts (marine_ais_msgs/AISContact).
///
/// AIS is a *sparse* position source: Class A vessels report every 2-10 s and
/// Class B every 30 s, so by the time the planner reads the costmap the last
/// fix is always stale. Painting the vessel at its last reported position is
/// therefore wrong in the dangerous direction -- at 10 kn a 30 s Class B gap is
/// roughly 150 m of error, and the planner would route confidently through
/// where the target actually is.
///
/// This layer instead replicates the interpolation strategy CAMP uses for
/// display, adapted for planning. Each contact is dead-reckoned forward from
/// its fix along course-over-ground at speed-over-ground, and the layer paints
/// the whole swept region *between* the last known position and the predicted
/// present position. The uncertainty envelope around that corridor grows with
/// time since fix, so a contact that has been quiet for a while occupies a
/// visibly larger, softer region rather than a crisp lie.
///
/// The asymmetry is deliberate. Over-painting costs a detour around empty
/// water; under-painting costs a collision. Where this layer must guess, it
/// guesses wide.
///
/// **Status gating.** Dead reckoning is applied only to contacts that could
/// plausibly be moving. A contact reporting at-anchor, moored or aground is
/// painted where it says it is: propagating it would inject pure error.
///
/// **Expiry is per class.** Class A (message types 1/2/3) and Class B (18/19)
/// have very different reporting rates, so a single timeout necessarily either
/// discards live Class A targets or retains stale Class B ghosts.
///
/// **A fading target is not a departing target.** With a shore-based receiver,
/// dropouts are expected and indistinguishable from a vessel leaving. Expiry is
/// therefore generous and uncertainty grows until it fires, rather than
/// contacts being deleted crisply. If the feed dies entirely, every contact
/// ages out and the layer simply stops contributing -- it never strands the
/// planner.
///
/// Aids to navigation (message type 21) are deliberately out of scope: the
/// tracker publishes those on a separate `atons` topic as bare points, and
/// where they are charted `s57_layer` already accounts for them.
class AISLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  AISLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void matchSize() override;
  void reset() override;
  bool isClearable() override {return true;}

  /// @brief Build the swept poses for one contact.
  ///
  /// Exposed for testing: this is the whole interpolation model, and it is
  /// worth exercising without standing up a costmap and a TF tree.
  ///
  /// @param contact   The contact, in world coordinates via @p origin.
  /// @param origin    Last known position in the costmap world frame.
  /// @param age       Seconds since the contact's fix.
  /// @return Swept poses ordered from the fix to the predicted position.
  std::vector<SweptPose> buildSweptPoses(
    const marine_ais_msgs::msg::AISContact & contact,
    const Point2D & origin,
    double age) const;

  /// @brief Seconds after which a contact of this message type is discarded.
  double maximumAgeFor(uint8_t position_message_id) const;

  /// @brief Whether a contact reporting this navigational status may be
  ///        dead-reckoned forward.
  static bool isDeadReckonable(uint8_t navigational_status);

protected:
  /// @brief Convert a geographic position to the costmap's world frame.
  /// @throws tf2::TransformException if the transform is unavailable.
  geometry_msgs::msg::Point llToWorld(const geographic_msgs::msg::GeoPoint & geo_point);

  void contactCallback(const marine_ais_msgs::msg::AISContact::ConstSharedPtr & msg);

  /// @brief Rasterise one swept pose into this layer's costmap.
  void paintSweptPose(
    const SweptPose & pose, double * min_x, double * min_y,
    double * max_x, double * max_y);

  rclcpp::Subscription<marine_ais_msgs::msg::AISContact>::SharedPtr contact_subscriber_;

  /// Latest contact per MMSI. AIS repeats a contact on every position report,
  /// so keying by id keeps exactly one live track per vessel.
  std::map<uint32_t, marine_ais_msgs::msg::AISContact> tracks_;
  std::mutex tracks_mutex_;

  std::string global_frame_id_;

  /// Extent painted on the previous cycle. Nav2 resets the master costmap only
  /// over the bounds the layers ask for, so a corridor we painted last cycle
  /// and no longer paint would persist in the master unless we keep asking for
  /// its old extent. Contacts move every cycle, so this is the normal case,
  /// not an edge case.
  bool has_last_bounds_{false};
  double last_min_x_{0.0};
  double last_min_y_{0.0};
  double last_max_x_{0.0};
  double last_max_y_{0.0};

  // --- parameters -------------------------------------------------------
  std::string topic_{"contacts"};
  double max_age_class_a_{90.0};
  double max_age_class_b_{180.0};
  double max_age_default_{120.0};
  double max_prediction_time_{120.0};
  double sigma_scale_{2.0};
  double default_position_sigma_{25.0};
  double default_speed_sigma_{0.5};
  double unknown_speed_{5.0};
  double default_vessel_radius_{10.0};
  double max_envelope_{500.0};
  int max_samples_{64};
  unsigned char contact_cost_{nav2_costmap_2d::LETHAL_OBSTACLE};
  unsigned char envelope_edge_cost_{1};
  /// Variance at or above which a covariance entry is treated as "unknown".
  /// Matches the tracker's `unknown_variance` parameter.
  double unknown_variance_threshold_{1.0e5};
};

// --- geometry helpers, free functions so tests can reach them ------------

/// @brief True if @p point lies inside @p polygon (crossing-number rule).
/// Winding-agnostic, so it does not care how the tracker orders hull vertices.
bool pointInPolygon(const Point2D & point, const std::vector<Point2D> & polygon);

/// @brief Distance from @p point to @p polygon; zero when inside.
double distanceToPolygon(const Point2D & point, const std::vector<Point2D> & polygon);

}  // namespace ais_layer

#endif  // AIS_LAYER__AIS_LAYER_HPP_
