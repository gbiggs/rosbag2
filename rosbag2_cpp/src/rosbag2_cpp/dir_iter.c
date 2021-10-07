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

#include <rosbag2_cpp_backport/dir_iter.h>

#include <rcutils/error_handling.h>
#include <rcutils/filesystem.h>

#include <errno.h>
#ifndef _WIN32
#include <dirent.h>
#include <unistd.h>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct rosbag2_cpp_dir_iter_state_t
{
#ifdef _WIN32
  HANDLE handle;
  WIN32_FIND_DATA data;
#else
  DIR * dir;
#endif
} rosbag2_cpp_dir_iter_state_t;

rosbag2_cpp_dir_iter_t *
rosbag2_cpp_dir_iter_start(const char * directory_path, const rcutils_allocator_t allocator)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(directory_path, NULL);
  RCUTILS_CHECK_ALLOCATOR_WITH_MSG(
    &allocator, "allocator is invalid", return NULL);

  rosbag2_cpp_dir_iter_t * iter = (rosbag2_cpp_dir_iter_t *)allocator.zero_allocate(
    1, sizeof(rosbag2_cpp_dir_iter_t), allocator.state);
  if (NULL == iter) {
    return NULL;
  }
  iter->allocator = allocator;

  rosbag2_cpp_dir_iter_state_t * state = (rosbag2_cpp_dir_iter_state_t *)allocator.zero_allocate(
    1, sizeof(rosbag2_cpp_dir_iter_state_t), allocator.state);
  if (NULL == state) {
    RCUTILS_SET_ERROR_MSG(
      "Failed to allocate memory.\n");
    goto rosbag2_cpp_dir_iter_start_fail;
  }
  iter->state = (void *)state;

#ifdef _WIN32
  char * search_path = rcutils_join_path(directory_path, "*", allocator);
  if (NULL == search_path) {
    goto rosbag2_cpp_dir_iter_start_fail;
  }
  state->handle = FindFirstFile(search_path, &state->data);
  allocator.deallocate(search_path, allocator.state);
  if (INVALID_HANDLE_VALUE == state->handle) {
    DWORD error = GetLastError();
    if (ERROR_FILE_NOT_FOUND != error || !rcutils_is_directory(directory_path)) {
      RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "Can't open directory %s. Error code: %d\n", directory_path, error);
      goto rosbag2_cpp_dir_iter_start_fail;
    }
  } else {
    iter->entry_name = state->data.cFileName;
  }
#else
  state->dir = opendir(directory_path);
  if (NULL == state->dir) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Can't open directory %s. Error code: %d\n", directory_path, errno);
    goto rosbag2_cpp_dir_iter_start_fail;
  }

  errno = 0;
  struct dirent * entry = readdir(state->dir);
  if (NULL != entry) {
    iter->entry_name = entry->d_name;
  } else if (0 != errno) {
    RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "Can't iterate directory %s. Error code: %d\n", directory_path, errno);
    goto rosbag2_cpp_dir_iter_start_fail;
  }
#endif

  return iter;

rosbag2_cpp_dir_iter_start_fail:
  rosbag2_cpp_dir_iter_end(iter);
  return NULL;
}

bool
rosbag2_cpp_dir_iter_next(rosbag2_cpp_dir_iter_t * iter)
{
  RCUTILS_CHECK_ARGUMENT_FOR_NULL(iter, false);
  rosbag2_cpp_dir_iter_state_t * state = (rosbag2_cpp_dir_iter_state_t *)iter->state;
  RCUTILS_CHECK_FOR_NULL_WITH_MSG(state, "iter is invalid", false);

#ifdef _WIN32
  if (FindNextFile(state->handle, &state->data)) {
    iter->entry_name = state->data.cFileName;
    return true;
  }
  FindClose(state->handle);
#else
  struct dirent * entry = readdir(state->dir);
  if (NULL != entry) {
    iter->entry_name = entry->d_name;
    return true;
  }
#endif

  iter->entry_name = NULL;
  return false;
}

void
rosbag2_cpp_dir_iter_end(rosbag2_cpp_dir_iter_t * iter)
{
  if (NULL == iter) {
    return;
  }

  rcutils_allocator_t allocator = iter->allocator;
  rosbag2_cpp_dir_iter_state_t * state = (rosbag2_cpp_dir_iter_state_t *)iter->state;
  if (NULL != state) {
#ifdef _WIN32
    FindClose(state->handle);
#else
    if (NULL != state->dir) {
      closedir(state->dir);
    }
#endif

    allocator.deallocate(state, allocator.state);
  }

  allocator.deallocate(iter, allocator.state);
}

#ifdef __cplusplus
}
#endif
