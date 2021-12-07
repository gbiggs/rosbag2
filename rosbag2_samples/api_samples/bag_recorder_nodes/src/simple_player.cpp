// Copyright 2021 Open Source Robotics Foundation
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

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

#include "rosbag2_cpp_backport/reader.hpp"
#include "rosbag2_cpp_backport/readers/sequential_reader.hpp"
#include "rosbag2_storage_backport/storage_options.hpp"
#include "rosbag2_transport_backport/player.hpp"
#include "rosbag2_transport_backport/play_options.hpp"


// The following main function just initializes a Player and makes it spin until
// the node is shutdown. Check the rosbag2_interfaces to send service calls to
// the Player and interact with it.
//
// Note that the player will always look for `my_bag` bag which is stored as a
// sqlite3 database.
//
// To exercise this interface, create a bag with a unique topic `my_topic` by
// running in a console:
//
// ```sh
// ros2 topic pub -t 1000 -r 1  /my_topic example_interfaces/msg/String '{data: "hello"}'
// ```
//
// While it is running, open another terminal and run:
//
// ```sh
// ros2 bag_bp record /my_topic -o my_bag
// ```
//
// Wait some time until a few messages are stored and stop both processes with
// Ctrl-C.
//
// You can playback the bag by running:
//
// ```sh
// ros2 run bag_recorder_nodes simple_player
// ```
//
// The Player node will start but will not publish anything. Once that's done,
// you can use `ros2 service call` to interact via the service API. For example,
// _play for 2.5 seconds_ would be:
//
// ```sh
// ros2 service call /rosbag2_player/play_for rosbag2_interfaces_backport/srv/PlayFor
// "{duration: {sec: 2, nanosec: 500000000}}"
// ```
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto reader = std::make_unique<rosbag2_cpp::Reader>(
    std::make_unique<rosbag2_cpp::readers::SequentialReader>());

  const rosbag2_storage::StorageOptions storage_options{ /* uri */ "my_bag",
    /* storage_id */ "sqlite3"};
  rosbag2_transport::PlayOptions play_options{};
  play_options.delay = 1.0;

  rclcpp::spin(
    std::make_shared<rosbag2_transport::Player>(
      std::move(reader), storage_options,
      play_options));

  rclcpp::shutdown();
  return 0;
}
