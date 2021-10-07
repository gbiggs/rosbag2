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

#ifndef ROSBAG2_CPP_BACKPORT__DIR_ITER_H_
#define ROSBAG2_CPP_BACKPORT__DIR_ITER_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcutils/allocator.h"
#include "rcutils/macros.h"

/// An iterator used for enumerating directory contents
typedef struct rosbag2_cpp_dir_iter_t
{
  /// The name of the enumerated file or directory
  const char * entry_name;
  /// The allocator used internally by iteration functions
  rcutils_allocator_t allocator;
  /// The platform-specific iteration state
  void * state;
} rosbag2_cpp_dir_iter_t;

/// Begin iterating over the contents of the specified directory.
/**
 * This function is used to list the files and directories that are contained in
 * a specified directory. The structure returned by it must be deallocated using
 * ::rosbag2_cpp_dir_iter_end when the iteration is completed. The name of the
 * enumerated entry is stored in the `entry_name` member of the returned object,
 * and the first entry is already populated upon completion of this function. To
 * populate the entry with the name of the next entry, use the
 * ::rosbag2_cpp_dir_iter_next function. Note that the "." and ".." entries are
 * typically among the entries enumerated.
 * \param[in] directory_path The directory path to iterate over the contents of.
 * \param[in] allocator Allocator used to create the returned structure.
 * \return An iterator object used to continue iterating directory contents
 * \return NULL if an error occurred
 */
rosbag2_cpp_dir_iter_t *
rosbag2_cpp_dir_iter_start(const char * directory_path, const rcutils_allocator_t allocator);

/// Continue iterating over the contents of a directory.
/**
 * \param[in] iter An iterator created by ::rosbag2_cpp_dir_iter_start.
 * \return `true` if another entry was found, or
 * \return `false` if there are no more entries in the directory.
 */
bool
rosbag2_cpp_dir_iter_next(rosbag2_cpp_dir_iter_t * iter);

/// Finish iterating over the contents of a directory.
/**
 * \param[in] iter An iterator created by ::rosbag2_cpp_dir_iter_start.
 */
void
rosbag2_cpp_dir_iter_end(rosbag2_cpp_dir_iter_t * iter);

#ifdef __cplusplus
}
#endif

#endif  // ROSBAG2_CPP_BACKPORT__DIR_ITER_H_
