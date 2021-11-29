// Copyright 2018, Bosch Software Innovations GmbH.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.Bosch Software Innovations GmbH.
#include <gmock/gmock.h>

#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_test_common_backport/subscription_manager.hpp"

#include "rosbag2_transport_backport/player.hpp"

#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"
#include "test_msgs/message_fixtures.hpp"

#include "qos.hpp"

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_test_fixture.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace std::chrono_literals;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

class RosBag2PlayForTestFixture : public RosBag2PlayTestFixture
{
public:
  static constexpr const char * kTopic1Name{"topic1"};
  static constexpr const char * kTopic2Name{"topic2"};
  static constexpr const char * kTopic1{"/topic1"};
  static constexpr const char * kTopic2{"/topic2"};

  std::vector<rosbag2_storage::TopicMetadata> get_topic_types()
  {
    return {{kTopic1Name, "test_msgs/BasicTypes", "", ""},
      {kTopic2Name, "test_msgs/Arrays", "", ""}};
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  get_serialized_messages()
  {
    auto primitive_message1 = get_messages_basic_types()[0];
    primitive_message1->int32_value = 42;

    auto complex_message1 = get_messages_arrays()[0];
    complex_message1->float32_values = {{40.0f, 2.0f, 0.0f}};
    complex_message1->bool_values = {{true, false, true}};

    std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>> messages =
    {serialize_test_message(kTopic1Name, 500, primitive_message1),
      serialize_test_message(kTopic1Name, 700, primitive_message1),
      serialize_test_message(kTopic1Name, 900, primitive_message1),
      serialize_test_message(kTopic2Name, 550, complex_message1),
      serialize_test_message(kTopic2Name, 750, complex_message1),
      serialize_test_message(kTopic2Name, 950, complex_message1)};
    return messages;
  }

protected:
  void SetUp() override
  {
    auto topic_types = get_topic_types();
    auto messages = get_serialized_messages();

    auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
    prepared_mock_reader->prepare(messages, topic_types);
    auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

    // Due to a problem related to the subscriber, we play many (3) messages but make the subscriber
    // node spin only until 2 have arrived. Hence the 2 as `launch_subscriber()` argument.
    sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1, 2);
    sub_->add_subscription<test_msgs::msg::Arrays>(kTopic2, 2);

    player_ = std::make_shared<rosbag2_transport::Player>(
      std::move(
        reader), storage_options_, play_options_);
  }

  std::shared_ptr<rosbag2_transport::Player> player_;
  std::future<void> await_received_messages_;
};

TEST_F(RosBag2PlayForTestFixture, play_for_recorded_messages_are_played_for_all_topics)
{
  auto await_received_messages = sub_->spin_subscriptions();

  player_->play_for(std::chrono::nanoseconds(std::chrono::milliseconds(1000)).count());

  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    kTopic1);
  EXPECT_THAT(replayed_test_primitives, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_primitives,
    Each(Pointee(Field(&test_msgs::msg::BasicTypes::int32_value, 42))));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    kTopic2);
  EXPECT_THAT(replayed_test_arrays, SizeIs(Ge(2u)));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::bool_values,
          ElementsAre(true, false, true)))));
  EXPECT_THAT(
    replayed_test_arrays,
    Each(
      Pointee(
        Field(
          &test_msgs::msg::Arrays::float32_values,
          ElementsAre(40.0f, 2.0f, 0.0f)))));
}

TEST_F(RosBag2PlayForTestFixture, play_for_none_are_played_due_to_duration)
{
  const rcutils_duration_value_t duration =
    std::chrono::nanoseconds(std::chrono::milliseconds(300)).count();

  auto await_received_messages = sub_->spin_subscriptions_for(duration);

  player_->play_for(duration);

  await_received_messages.get();

  auto replayed_test_primitives = sub_->get_received_messages<test_msgs::msg::BasicTypes>(
    kTopic1);
  EXPECT_THAT(replayed_test_primitives, SizeIs(Le(0u)));

  auto replayed_test_arrays = sub_->get_received_messages<test_msgs::msg::Arrays>(
    kTopic2);
  EXPECT_THAT(replayed_test_arrays, SizeIs(Le(0u)));
}
