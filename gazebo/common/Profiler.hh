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

/*
  This is a backport of ignition/common/Profiler.hh
*/


#ifndef GAZEBO_COMMON_PROFILER_HH_
#define GAZEBO_COMMON_PROFILER_HH_

#include <memory>
#include <string>

#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/Remotery/lib/Remotery.h"
#include "gazebo/util/system.hh"

/// \brief Explicit instantiation for typed SingletonT.
GZ_SINGLETON_DECLARE(GZ_COMMON_VISIBLE, gazebo, common, Profiler)

namespace gazebo
{
  namespace common
  {
    class ProfilerImpl;

    /// \brief Used to perform application-wide performance profiling
    ///
    /// This class provides the necessary infrastructure for recording profiling
    /// information of an application while it is running. The actual profiler
    /// implementation can be choosen at runtime, and is invoked through a
    /// series of macros found below.
    ///
    /// In general, users shouldn't directly interface with this class,
    /// but instead use the profiling macros, which can be enabled/disabled
    /// at compile time, which eliminates any performance impact of profiling.
    ///
    /// Profiler is enabled by setting IGN_ENABLE_PROFILER at compile time.
    ///
    /// The profiler header also exports several convenience macros to make
    /// adding inspection points easier.
    ///
    /// * GZ_PROFILE_THREAD_NAME - Set the name of the current profiled thread.
    /// * GZ_PROFILE_LOG_TEXT - Log text to the profiler console (if supported)
    /// * GZ_PROFILE_BEGIN - Begin a named profile sample
    /// * GZ_PROFILE_END - End a named profile sample
    /// * GZ_PROFILE - RAII-style profile sample. The sample will end at the
    ///     end of the current scope.
    class GZ_COMMON_VISIBLE Profiler
        : public virtual SingletonT<Profiler>
    {
      /// \brief Constructor
      protected: Profiler();

      /// \brief Destructor
      protected: ~Profiler();

      /// \brief Set the name of the current thread
      /// \param[in] _name Name to set
      public: void SetThreadName(const char *_name);

      /// \brief Log text to profiler output (if supported)
      /// If the underlying profiler implementation supports additional
      /// log messages, this can be used to send.
      ///
      /// Currently, the Remotery implentation supports this functionality.
      /// \param[in] _text Text to log.
      public: void LogText(const char *_text);

      /// \brief Begin a named profiling sample.
      /// Begins a CPU profiler sample with a given name. Can optionally take
      /// a hash parameter to be cached between executions of `BeginSample`, so
      /// that hashes don't need to be recomputed.
      /// \param[in] _name Name of the sample
      /// \param[in,out] _hash An optional hash value that can be cached
      ///   between executions.
      public: void BeginSample(const char *_name, uint32_t *_hash = nullptr);

      /// \brief End a profiling sample.
      public: void EndSample();

      /// \brief Get the underlying profiler implentation name
      public: std::string ImplementationName() const;

      /// \brief Detect if profiler is enabled and has an implementation
      public: bool Valid() const;

      /// \brief Pointer to the profiler implementation
      private: ProfilerImpl *impl;

      /// \brief Needed for SingletonT.
      private: friend class SingletonT<Profiler>;
    };

    /// \brief Used to provide C++ RAII-style profiling sample.
    /// The sample will start on the construction of the `ScopedProfile` object
    /// and stop when the object leaves scope.
    class GZ_COMMON_VISIBLE ScopedProfile
    {
      /// \brief Constructor. Starts profile sample.
      /// \param[in] _name Name of the sample
      /// \param[in,out] _hash An optional hash value that can be cached
      ///   between executions.
      public: ScopedProfile(const char *_name, uint32_t *_hash)
      {
        Profiler::Instance()->BeginSample(_name, _hash);
      }

      /// \brief Destructor. Stops profile sample.
      public: ~ScopedProfile()
      {
        Profiler::Instance()->EndSample();
      }
    };
  }
}

#ifndef GZ_PROFILER_ENABLE
/// Always set this variable to some value
#define GZ_PROFILER_ENABLE 0
#endif

#if GZ_PROFILER_ENABLE
/// \brief Set name of profiled thread
#define GZ_PROFILE_THREAD_NAME(name) \
    gazebo::common::Profiler::Instance()->SetThreadName(name);
/// \brief Log profiling text, if supported by implementation
#define GZ_PROFILE_LOG_TEXT(name) \
    gazebo::common::Profiler::Instance()->LogText(name);
/// \brief Being profiling sample
#define GZ_PROFILE_BEGIN(name) \
    gazebo::common::Profiler::Instance()->BeginSample(name)
/// \brief End profiling sample
#define GZ_PROFILE_END() \
    gazebo::common::Profiler::Instance()->EndSample()

/// \brief Convenience wrapper for scoped profiling sample. Use GZ_PROFILE
#define GZ_PROFILE_L(name, line) \
static uint32_t __hash##line = 0; \
gazebo::common::ScopedProfile __profile##line(name, &__hash##line);
/// \brief Scoped profiling sample. Sample will stop at end of scope.
#define GZ_PROFILE(name)             GZ_PROFILE_L(name, __LINE__);

#else

#define GZ_PROFILE_THREAD_NAME(name) ((void) name)
#define GZ_PROFILE_LOG_TEXT(name)    ((void) name)
#define GZ_PROFILE_BEGIN(name)       ((void) name)
#define GZ_PROFILE_END()             ((void) 0)
#define GZ_PROFILE_L(name, line)     ((void) name)
#define GZ_PROFILE(name)             ((void) name)
#endif  // GZ_PROFILER_ENABLE

/// \brief Macro to determine if profiler is enabled and has an implementation.
#define GZ_PROFILER_VALID \
    GZ_PROFILER_ENABLE && gazebo::common::Profiler::Instance()->Valid()

#endif  // GAZEBO_COMMON_PROFILER_HH_
