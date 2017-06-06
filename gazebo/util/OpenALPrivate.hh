/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_UTIL_OPENAL_PRIVATE_HH_
#define _GAZEBO_UTIL_OPENAL_PRIVATE_HH_

#include <string>
#include <vector>

#include "gazebo/gazebo_config.h"
#include "gazebo/util/UtilTypes.hh"

#ifdef HAVE_OPENAL

struct ALCcontext_struct;
struct ALCdevice_struct;

namespace gazebo
{
  namespace util
  {
    /// \internal
    /// \brief Private dat for OpenAL
    class OpenALPrivate
    {
      /// \brief OpenAL audio context pointer.
      public: ALCcontext_struct *context;

      /// \brief OpenAL audio device pointer.
      public: ALCdevice_struct *audioDevice;

      /// \brief OpenAL sink pointer.
      public: OpenALSinkPtr sink;
    };

    /// \internal
    /// \brief Private data for OpenALSource
    class OpenALSourcePrivate
    {
      /// \brief OpenAL source index.
      public: unsigned int alSource;

      /// \brief OpenAL buffer index.
      public: unsigned int alBuffer;

      /// \brief Names of collision objects that should trigger audio
      /// playback.
      public: std::vector<std::string> collisionNames;
    };
  }
}
#endif
#endif
