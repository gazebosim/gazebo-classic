/*
 * Copyright (C) 2018 Open Source Robotics Foundation
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

#ifndef IGNITION_COMMON_PROFILERIMPL_HH_
#define IGNITION_COMMON_PROFILERIMPL_HH_

#include <string>

namespace ignition
{
  namespace common
  {
    /// \brief Interface to be implemented by profiler implementations.
    class ProfilerImpl
    {
      /// \brief Constructor.
      public: ProfilerImpl() = default;

      /// \brief Destructor.
      public: virtual ~ProfilerImpl() = default;

      /// \brief Retrieve profiler name.
      public: virtual std::string Name() const = 0;

      /// \brief Set the name of the current thread
      /// \param[in] _name Name to set
      public: virtual void SetThreadName(const char *_name) = 0;

      /// \brief Log text to profiler output (if supported)
      /// If the underlying profiler implementation supports additional
      /// log messages, this can be used to send.
      /// \param[in] _text Text to log.
      public: virtual void LogText(const char *_text) = 0;

      /// \brief Begin a named profiling sample.
      /// Begins a CPU profiler sample with a given name. Can optionally take
      /// a hash parameter to be cached between executions of `BeginSample`, so
      /// that hashes don't need to be recomputed.
      /// \param[in] _name Name of the sample
      /// \param[in,out] _hash An optional hash value that can be cached
      ///   between executions.
      public: virtual void BeginSample(const char *_name, uint32_t *_hash) = 0;

      /// \brief End a profiling sample.
      public: virtual void EndSample() = 0;
    };
  }
}

#endif  // IGNITION_COMMON_PROFILERIMPL_HH_
