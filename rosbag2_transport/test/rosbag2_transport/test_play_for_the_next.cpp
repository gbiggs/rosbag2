// Copyright 2021 Open Source Robotics Foundation, Inc.
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
// limitations under the License.
#include <gmock/gmock.h>

#include <chrono>
#include <future>
#include <memory>
#include <utility>
#include <vector>

#include "rosbag2_play_test_fixture.hpp"
#include "rosbag2_transport_backport/player.hpp"
#include "test_msgs/message_fixtures.hpp"
#include "test_msgs/msg/arrays.hpp"
#include "test_msgs/msg/basic_types.hpp"

using namespace ::testing;  // NOLINT
using namespace rosbag2_transport;  // NOLINT
using namespace rosbag2_test_common;  // NOLINT

constexpr rcutils_duration_value_t MilliSecondsToDuration(uint64_t milliseconds)
{
  return std::chrono::nanoseconds(milliseconds * 1000000u).count();
}

class MockPlayer : public rosbag2_transport::Player
{
public:
  MockPlayer(
    std::unique_ptr<rosbag2_cpp::Reader> reader,
    const rosbag2_storage::StorageOptions & storage_options,
    const rosbag2_transport::PlayOptions & play_options)
  : Player(std::move(reader), storage_options, play_options)
  {}

  std::vector<rclcpp::PublisherBase *> get_list_of_publishers()
  {
    std::vector<rclcpp::PublisherBase *> pub_list;
    for (const auto & publisher : publishers_) {
      pub_list.push_back(static_cast<rclcpp::PublisherBase *>(publisher.second.get()));
    }
    return pub_list;
  }

  void wait_for_playback_to_start()
  {
    while (!playing_messages_from_queue_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
};

class RosBag2PlayForTheNextTestFixture : public RosBag2PlayTestFixture
{
public:
  static constexpr int kIntValue{32};

  static constexpr float kFloat1Value{40.};
  static constexpr float kFloat2Value{2.};
  static constexpr float kFloat3Value{0.};

  static constexpr bool kBool1Value{false};
  static constexpr bool kBool2Value{true};
  static constexpr bool kBool3Value{false};

  static constexpr const char * kTopic1Name{"topic1"};
  static constexpr const char * kTopic2Name{"topic2"};
  static constexpr const char * kTopic1{"/topic1"};
  static constexpr const char * kTopic2{"/topic2"};

  std::vector<rosbag2_storage::TopicMetadata> get_topic_types()
  {
    return {{kTopic1Name, "test_msgs/BasicTypes", "", ""},
      {kTopic2Name, "test_msgs/Arrays", "", ""}};
  }

  std::vector<rosbag2_storage::TopicMetadata> get_topic_type()
  {
    return {{kTopic1Name, "test_msgs/BasicTypes", "", ""}};
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  get_serialized_messages_for_one_topic()
  {
    auto primitive_message = get_messages_basic_types()[0];
    primitive_message->int32_value = kIntValue;

    // @{ Ordering matters. The mock reader implementation moves messages
    //    around without any knowledge about message chronology. It just picks
    //    the next one Make sure to keep the list in order or sort it!
    return std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
           {serialize_test_message(kTopic1Name, 1000, primitive_message),
             serialize_test_message(kTopic1Name, 1500, primitive_message),
             serialize_test_message(kTopic1Name, 2000, primitive_message),
             serialize_test_message(kTopic1Name, 2500, primitive_message)};
    // @}
  }

  std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
  get_serialized_messages_for_two_topics()
  {
    auto primitive_message = get_messages_basic_types()[0];
    primitive_message->int32_value = kIntValue;

    auto complex_message = get_messages_arrays()[0];
    complex_message->float32_values = {{kFloat1Value, kFloat2Value, kFloat3Value}};
    complex_message->bool_values = {{kBool1Value, kBool2Value, kBool3Value}};

    // @{ Ordering matters. The mock reader implementation moves messages
    //    around without any knowledge about message chronology. It just picks
    //    the next one Make sure to keep the list in order or sort it!
    return std::vector<std::shared_ptr<rosbag2_storage::SerializedBagMessage>>
           {serialize_test_message(kTopic2Name, 500, complex_message),
             serialize_test_message(kTopic1Name, 1000, primitive_message),
             serialize_test_message(kTopic2Name, 1400, complex_message),
             serialize_test_message(kTopic1Name, 1500, primitive_message),
             serialize_test_message(kTopic2Name, 2000, complex_message),
             serialize_test_message(kTopic1Name, 2000, primitive_message),
             serialize_test_message(kTopic1Name, 2500, primitive_message),
             serialize_test_message(kTopic2Name, 2700, complex_message)};
    // @}
  }
};

TEST_F(RosBag2PlayForTheNextTestFixture, play_for_the_next_with_false_preconditions) {
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(get_serialized_messages_for_one_topic(), get_topic_type());
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  ASSERT_FALSE(player->is_paused());
  ASSERT_FALSE(player->play_for_the_next(0));
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(1200u)));

  player->pause();

  ASSERT_TRUE(player->is_paused());
  ASSERT_FALSE(player->play_for_the_next(0));
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(2000u)));
}

TEST_F(RosBag2PlayForTheNextTestFixture, play_for_the_next_duration_which_plays_all_messages) {
  auto messages = get_serialized_messages_for_one_topic();

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, get_topic_type());
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1, messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();

  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(3000u)));
  ASSERT_TRUE(player->is_paused());

  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(kTopic1);
  EXPECT_THAT(replayed_topic1, SizeIs(4u));
}

TEST_F(RosBag2PlayForTheNextTestFixture, play_for_the_next_duration_which_plays_some_messages) {
  auto messages = get_serialized_messages_for_one_topic();

  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(messages, get_topic_type());
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1, messages.size());

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();

  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  // Should play only one message.
  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(1200u)));
  ASSERT_TRUE(player->is_paused());

  // Should not play any message.
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(100u)));
  ASSERT_TRUE(player->is_paused());

  // Should play one message.
  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(300u)));
  ASSERT_TRUE(player->is_paused());

  // Continues with the rest of the messages.
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(kTopic1);
  EXPECT_THAT(replayed_topic1, SizeIs(4u));
}


TEST_F(RosBag2PlayForTheNextTestFixture, play_for_the_next_duration_with_filtered_topics) {
  auto prepared_mock_reader = std::make_unique<MockSequentialReader>();
  prepared_mock_reader->prepare(get_serialized_messages_for_two_topics(), get_topic_types());
  auto reader = std::make_unique<rosbag2_cpp::Reader>(std::move(prepared_mock_reader));

  // Skips topic2, allows only topic1.
  play_options_.topics_to_filter = {kTopic1Name};
  auto player = std::make_shared<MockPlayer>(std::move(reader), storage_options_, play_options_);

  sub_ = std::make_shared<SubscriptionManager>();
  sub_->add_subscription<test_msgs::msg::BasicTypes>(kTopic1, 4);
  sub_->add_subscription<test_msgs::msg::Arrays>(kTopic2, 0);

  // Wait for discovery to match publishers with subscribers
  ASSERT_TRUE(
    sub_->spin_and_wait_for_matched(player->get_list_of_publishers(), std::chrono::seconds(30)));

  auto await_received_messages = sub_->spin_subscriptions();

  player->pause();

  ASSERT_TRUE(player->is_paused());

  auto player_future = std::async(std::launch::async, [&player]() -> void {player->play();});
  player->wait_for_playback_to_start();

  // Should not play any message. Time cursor: 600ms
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(600u)));
  ASSERT_TRUE(player->is_paused());

  // Should play only one message. Time cursor: 1100ms
  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(500u)));
  ASSERT_TRUE(player->is_paused());

  // Should not play any message. Time cursor: 1450ms
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(350u)));
  ASSERT_TRUE(player->is_paused());

  // Should play only one message. Time cursor: 1550ms
  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(100u)));
  ASSERT_TRUE(player->is_paused());

  // Should not play any message because there are none. Time cursor: 1950ms
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(400u)));
  ASSERT_TRUE(player->is_paused());

  // Should play one message from topic1 only. Time cursor: 2050ms
  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(100u)));
  ASSERT_TRUE(player->is_paused());

  // Should play one message. Time cursor: 2650ms
  ASSERT_TRUE(player->play_for_the_next(MilliSecondsToDuration(600u)));
  ASSERT_TRUE(player->is_paused());

  // Should not play any message. Time cursor: 2850ms
  ASSERT_FALSE(player->play_for_the_next(MilliSecondsToDuration(200u)));
  ASSERT_TRUE(player->is_paused());

  // Continues with the rest of the messages.
  player->resume();
  player_future.get();
  await_received_messages.get();

  auto replayed_topic1 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(kTopic1);
  EXPECT_THAT(replayed_topic1, SizeIs(4u));

  auto replayed_topic2 = sub_->get_received_messages<test_msgs::msg::BasicTypes>(kTopic2);
  EXPECT_THAT(replayed_topic2, SizeIs(0u));
}
