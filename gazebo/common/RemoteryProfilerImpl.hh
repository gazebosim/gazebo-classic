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

#ifndef GAZEBO_COMMON_REMOTERYPROFILERIMPL_HH_
#define GAZEBO_COMMON_REMOTERYPROFILERIMPL_HH_

#include <string>

#include "gazebo/common/RemoteryConfig.h"
#include <gazebo/common/Remotery/lib/Remotery.h>

#include "ProfilerImpl.hh"

namespace gazebo
{
  namespace common
  {
    /// \brief Remotery profiler implementation
    ///
    /// Used to provide Remotery (https://github.com/Celtoys/Remotery) profiler
    /// implementation.
    ///
    /// Remotery will start and open a web socket that will allow users to view
    /// live profiling information via the web browser.
    ///
    /// The Remotery profiler can additionally be configured via environment
    /// variables at runtime.
    ///
    /// * RMT_PORT: Port to listen for incoming connections on.
    /// * RMT_QUEUE_SIZE: Size of the internal message queues
    /// * RMT_MSGS_PER_UPDATE: Upper limit on messages consumed per loop
    /// * RMT_SLEEP_BETWEEN_UPDATES: Controls profile server update rate.
    class RemoteryProfilerImpl: public ProfilerImpl
    {
      /// \brief Constructor.
      public: RemoteryProfilerImpl();

      /// \brief Destructor.
      public: ~RemoteryProfilerImpl() final;

      /// \brief Retrieve profiler name.
      public: std::string Name() const final;

      /// \brief Set the name of the current thread
      /// \param[in] _name Name to set
      public: void SetThreadName(const char *_name) final;

      /// \brief Log text to profiler output.
      /// Will appear in the Remotery console.
      /// \param[in] _text Text to log.
      public: void LogText(const char *_text) final;

      /// \brief Begin a named profiling sample.
      /// Begins a CPU profiler sample with a given name. Can optionally take
      /// a hash parameter to be cached between executions of `BeginSample`, so
      /// that hashes don't need to be recomputed.
      /// \param[in] _name Name of the sample
      /// \param[in,out] _hash An optional hash value that can be cached
      ///   between executions.
      public: void BeginSample(const char *_name, uint32_t *_hash) final;

      /// \brief End a profiling sample.
      public: void EndSample() final;

      /// \brief Handle input coming from Remotery web console.
      /// \param[in] _text Incoming input.
      public: void HandleInput(const char *_text);

      /// \brief Remotery settings.
      private: rmtSettings *settings;

      /// \brief Remotery instance.
      private: Remotery *rmt;
    };
  }
}

#endif  // GAZEBO_COMMON_PROFILERIMPL_HH_
