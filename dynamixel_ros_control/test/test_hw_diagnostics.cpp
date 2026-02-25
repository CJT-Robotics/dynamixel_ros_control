// Copyright (c) 2024 Team Hector, TU Darmstadt
// SPDX-License-Identifier: BSD-3-Clause

#include "test_hardware_interface_common.hpp"

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace dynamixel_ros_control::test {

// ============================================================================
// Diagnostics Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, Diagnostics_PublishedOnTopic)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_NE(msg, nullptr) << "No diagnostics message received";
  ASSERT_FALSE(msg->status.empty());
  EXPECT_EQ(msg->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(msg->status[0].message, "OK");
}

TEST_F(HardwareInterfaceTest, Diagnostics_ContainsExpectedFields)
{
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_NE(msg, nullptr);
  ASSERT_FALSE(msg->status.empty());

  const auto& values = msg->status[0].values;
  std::set<std::string> found_keys;
  for (const auto& kv : values) {
    found_keys.insert(kv.key);
  }
  EXPECT_TRUE(found_keys.count("e_stop_active")) << "Missing e_stop_active";
  EXPECT_TRUE(found_keys.count("torque_enabled")) << "Missing torque_enabled";
  EXPECT_TRUE(found_keys.count("consecutive_read_errors")) << "Missing consecutive_read_errors";
  EXPECT_TRUE(found_keys.count("consecutive_write_errors")) << "Missing consecutive_write_errors";
}

TEST_F(HardwareInterfaceTest, Diagnostics_ReflectsEStopState)
{
  // Initially should be OK with e_stop_active=false
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);
  EXPECT_EQ(msg->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::OK);

  std::string e_stop_value;
  for (const auto& kv : msg->status[0].values) {
    if (kv.key == "e_stop_active")
      e_stop_value = kv.value;
  }
  EXPECT_EQ(e_stop_value, "false");

  // Activate e-stop
  auto estop_pub = createEStopPublisher();
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(500ms);

  // Wait for diagnostics to reflect e-stop
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
    if (msg && !msg->status.empty() && msg->status[0].level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
      break;
    }
  }

  ASSERT_NE(msg, nullptr);
  ASSERT_FALSE(msg->status.empty());
  EXPECT_EQ(msg->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
  EXPECT_EQ(msg->status[0].message, "E-Stop active");

  for (const auto& kv : msg->status[0].values) {
    if (kv.key == "e_stop_active") {
      EXPECT_EQ(kv.value, "true");
    }
  }

  // Cleanup
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

TEST_F(HardwareInterfaceTest, Diagnostics_ReflectsTorqueState)
{
  // Initially torque should be enabled
  diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10).transient_local(),
      [&msg](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_NE(msg, nullptr);

  std::string torque_value;
  for (const auto& kv : msg->status[0].values) {
    if (kv.key == "torque_enabled")
      torque_value = kv.value;
  }
  EXPECT_EQ(torque_value, "true");

  // Disable torque
  auto torque_client = createTorqueClient();
  ASSERT_TRUE(setTorque(torque_client, false));
  std::this_thread::sleep_for(500ms);

  // Wait for diagnostics to reflect torque off
  deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
    if (msg) {
      for (const auto& kv : msg->status[0].values) {
        if (kv.key == "torque_enabled" && kv.value == "false") {
          goto found;
        }
      }
    }
  }
found:

  ASSERT_NE(msg, nullptr);
  for (const auto& kv : msg->status[0].values) {
    if (kv.key == "torque_enabled") {
      EXPECT_EQ(kv.value, "false");
    }
  }

  // Re-enable torque
  ASSERT_TRUE(setTorque(torque_client, true));
}

TEST_F(HardwareInterfaceTest, Diagnostics_OnlyPublishedOnChange)
{
  // Wait for the initial diagnostics message (published on first state)
  std::vector<diagnostic_msgs::msg::DiagnosticArray::SharedPtr> messages;
  auto sub = tester_node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
      "/athena_arm_interface/diagnostics", rclcpp::QoS(10).transient_local(),
      [&messages](diagnostic_msgs::msg::DiagnosticArray::SharedPtr m) { messages.push_back(m); });

  // Wait for the first message
  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (messages.empty() && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  ASSERT_GE(messages.size(), 1u) << "Expected at least one initial diagnostics message";

  // Record count, then wait — no further messages should arrive since nothing changed
  size_t count_after_initial = messages.size();
  std::this_thread::sleep_for(1s);
  for (int i = 0; i < 20; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }
  EXPECT_EQ(messages.size(), count_after_initial)
      << "No additional diagnostics should be published when state is unchanged";

  // Now trigger a state change (e-stop) and verify a new message arrives
  auto estop_pub = createEStopPublisher();
  std_msgs::msg::Bool estop_msg;
  estop_msg.data = true;
  estop_pub->publish(estop_msg);

  deadline = std::chrono::steady_clock::now() + 3s;
  while (messages.size() <= count_after_initial && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  EXPECT_GT(messages.size(), count_after_initial) << "A new diagnostics message should be published on state change";
  EXPECT_EQ(messages.back()->status[0].level, diagnostic_msgs::msg::DiagnosticStatus::WARN);

  // Cleanup
  estop_msg.data = false;
  estop_pub->publish(estop_msg);
  std::this_thread::sleep_for(300ms);
}

// ============================================================================
// Goal Joint State Publishing Tests
// ============================================================================

TEST_F(HardwareInterfaceTest, GoalJointStates_Published)
{
  sensor_msgs::msg::JointState::SharedPtr msg;
  auto sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/athena_arm_interface/goal_joint_states", 10, [&msg](sensor_msgs::msg::JointState::SharedPtr m) { msg = m; });

  auto deadline = std::chrono::steady_clock::now() + 10s;
  while (!msg && std::chrono::steady_clock::now() < deadline) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  ASSERT_NE(msg, nullptr) << "No goal joint state message received";
  EXPECT_GE(msg->name.size(), 7u) << "Should have at least 7 arm joint names";
  EXPECT_EQ(msg->position.size(), msg->name.size());
  EXPECT_EQ(msg->velocity.size(), msg->name.size());
  EXPECT_EQ(msg->effort.size(), msg->name.size());

  std::set<std::string> names(msg->name.begin(), msg->name.end());
  EXPECT_TRUE(names.count("arm_joint_1")) << "Missing arm_joint_1";
  EXPECT_TRUE(names.count("arm_joint_7")) << "Missing arm_joint_7";
}

TEST_F(HardwareInterfaceTest, GoalJointStates_ReflectsCommandedPosition)
{
  sensor_msgs::msg::JointState::SharedPtr goal_msg;
  auto goal_sub = tester_node_->create_subscription<sensor_msgs::msg::JointState>(
      "/athena_arm_interface/goal_joint_states", 10,
      [&goal_msg](sensor_msgs::msg::JointState::SharedPtr m) { goal_msg = m; });

  // Load controller and command a position
  ASSERT_TRUE(loadAndActivateController("arm_position_controller"));

  auto pub = tester_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/arm_position_controller/commands", 10);
  auto deadline = std::chrono::steady_clock::now() + 5s;
  while (std::chrono::steady_clock::now() < deadline && pub->get_subscription_count() == 0) {
    std::this_thread::sleep_for(50ms);
    executor_->spin_some();
  }
  ASSERT_GT(pub->get_subscription_count(), 0u);

  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  pub->publish(cmd);
  std::this_thread::sleep_for(2s);

  // Spin to receive latest messages
  for (int i = 0; i < 20; ++i) {
    executor_->spin_some();
    std::this_thread::sleep_for(50ms);
  }

  // Check goal joint states reflect the commanded position
  ASSERT_NE(goal_msg, nullptr) << "No goal joint state received";
  for (size_t i = 0; i < goal_msg->name.size(); ++i) {
    if (goal_msg->name[i].find("arm_joint") != std::string::npos) {
      EXPECT_NEAR(goal_msg->position[i], 0.5, 0.15)
          << "Joint " << goal_msg->name[i] << " goal position should be near target";
    }
  }
}

}  // namespace dynamixel_ros_control::test

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
