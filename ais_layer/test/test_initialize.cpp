// Copyright (c) 2026 Roland Arsenault
// Licensed under BSD license

// Runs AISLayer::initialize() on a real lifecycle node, so the parameter
// declaration and the subscription are exercised on every distro this
// package is built for, not only compiled.

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "ais_layer/ais_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

// Nav2's lifecycle node moved from nav2_util (Jazzy) to nav2_ros_common
// (Rolling); Layer::initialize takes a weak pointer to whichever one exists.
#if __has_include("nav2_ros_common/lifecycle_node.hpp")
#include "nav2_ros_common/lifecycle_node.hpp"
using TestLifecycleNode = nav2::LifecycleNode;
#else
#include "nav2_util/lifecycle_node.hpp"
using TestLifecycleNode = nav2_util::LifecycleNode;
#endif

namespace
{

constexpr char kLayerName[] = "ais";

/// Exposes what onInitialize() sets, so the tests can check it.
class InitTestLayer : public ais_layer::AISLayer
{
public:
  using AISLayer::contact_subscriber_;
  using AISLayer::max_age_class_a_;
  using AISLayer::topic_;
};

class InitializeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  /// A lifecycle node carrying the given parameter overrides.
  static std::shared_ptr<TestLifecycleNode> makeNode(
    const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    return std::make_shared<TestLifecycleNode>("ais_layer_initialize_test", "", options);
  }

  /// Calls initialize() as Nav2's costmap does, with no callback group.
  void initialize(InitTestLayer & layer, const std::shared_ptr<TestLifecycleNode> & node)
  {
    layer.initialize(&layered_costmap_, kLayerName, tf_.get(), node, nullptr);
  }

  void SetUp() override
  {
    tf_ = std::make_unique<tf2_ros::Buffer>(std::make_shared<rclcpp::Clock>());
  }

  nav2_costmap_2d::LayeredCostmap layered_costmap_{"map", false, false};
  std::unique_ptr<tf2_ros::Buffer> tf_;
};

}  // namespace

TEST_F(InitializeTest, ReadsParameterOverrides)
{
  auto node = makeNode(
    {rclcpp::Parameter("ais.topic", "ais_contacts"),
      rclcpp::Parameter("ais.max_age_class_a", 123.0)});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  EXPECT_EQ("ais_contacts", layer.topic_);
  EXPECT_DOUBLE_EQ(123.0, layer.max_age_class_a_);
  EXPECT_EQ("ais_contacts", node->get_parameter("ais.topic").as_string());
  // Parameters without an override are declared with the layer's defaults.
  EXPECT_TRUE(node->has_parameter("ais.max_age_class_b"));
  EXPECT_TRUE(node->get_parameter("ais.enabled").as_bool());
  EXPECT_TRUE(layer.isEnabled());
}

TEST_F(InitializeTest, KeepsParameterDeclaredBeforeInitialize)
{
  auto node = makeNode({});
  node->declare_parameter("ais.enabled", false);
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  EXPECT_FALSE(node->get_parameter("ais.enabled").as_bool());
  EXPECT_FALSE(layer.isEnabled());
}

TEST_F(InitializeTest, SecondInitializeDoesNotThrow)
{
  auto node = makeNode({rclcpp::Parameter("ais.topic", "ais_contacts")});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));
  // Every parameter is already declared now; declaring one again would throw.
  ASSERT_NO_THROW(initialize(layer, node));
  EXPECT_EQ("ais_contacts", layer.topic_);
}

TEST_F(InitializeTest, SubscribesOnTheConfiguredTopic)
{
  auto node = makeNode({rclcpp::Parameter("ais.topic", "ais_contacts")});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  ASSERT_NE(nullptr, layer.contact_subscriber_);
  EXPECT_STREQ("/ais_contacts", layer.contact_subscriber_->get_topic_name());
  EXPECT_EQ(1u, node->count_subscribers("/ais_contacts"));
}

TEST_F(InitializeTest, SubscribesOnTheDefaultTopicWithoutOverride)
{
  auto node = makeNode({});
  InitTestLayer layer;
  ASSERT_NO_THROW(initialize(layer, node));

  ASSERT_NE(nullptr, layer.contact_subscriber_);
  EXPECT_STREQ("/contacts", layer.contact_subscriber_->get_topic_name());
}
