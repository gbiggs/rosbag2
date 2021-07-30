/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef ROSBAG2_CPP__READER_INTERFACES__SINGLE_BAG_OPENER_INTERFACE_HPP_
#define ROSBAG2_CPP__READER_INTERFACES__SINGLE_BAG_OPENER_INTERFACE_HPP_

#include <memory>
#include <vector>

#include "rosbag2_cpp/converter_options.hpp"
#include "rosbag2_cpp/visibility_control.hpp"

#include "rosbag2_storage/bag_metadata.hpp"
#include "rosbag2_storage/serialized_bag_message.hpp"
#include "rosbag2_storage/storage_filter.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "rosbag2_storage/topic_metadata.hpp"

namespace rosbag2_cpp
{
namespace reader_interfaces
{

class ROSBAG2_CPP_PUBLIC SingleBagOpenerInterface : public BaseReaderInterface
{
public:
  SingleBagOpenerInterface(
    rosbag2_storage::StorageOptions storage_options,
    ConverterOptions converter_options)
    : BaseReaderInterface()
  {}

  virtual ~SingleBagOpenerInterface() {}
};

}  // namespace reader_interfaces
}  // namespace rosbag2_cpp

#endif  // ROSBAG2_CPP__READER_INTERFACES__SINGLE_BAG_OPENER_INTERFACE_HPP_
