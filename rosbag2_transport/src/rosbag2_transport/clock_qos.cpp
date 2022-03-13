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

#include "rosbag2_transport_backport/clock_qos.hpp"

namespace rosbag2_transport
{

ClockQoS::ClockQoS(const rclcpp::QoSInitialization & qos_initialization)
// Using `rmw_qos_profile_sensor_data` intentionally.
// It's best effort and `qos_initialization` is overriding the depth to 1.
: QoS(qos_initialization, rmw_qos_profile_default)
{}

}  // namespace rosbag2_transport
