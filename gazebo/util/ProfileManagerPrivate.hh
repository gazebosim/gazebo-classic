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


#include "gazebo/util/Profiler.hh"
#include "gazebo/util/ProfileNode.hh"
#include "gazebo/common/Time.hh"

namespace gazebo {
  namespace util {
    /// \brief Private data for the ProfileManager class
    class ProfileManagerPrivate {
      /// \brief Counter for number of frame visits
      public: int FrameCounter;

      /// \brief Time at which the profiler manager was restarted
      public: gazebo::common::Time ResetTime;

      /// \brief Root Profile Node for each potential thread
      public: ProfileNode gRoots[PROFILER_MAX_THREAD_COUNT] = {
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr),
        ProfileNode("Root", nullptr), ProfileNode("Root", nullptr)
      };

      /// \brief Current profile node for each potential thread.
      public: ProfileNode* gCurrentNodes[PROFILER_MAX_THREAD_COUNT] = {
        &gRoots[ 0], &gRoots[ 1], &gRoots[ 2], &gRoots[ 3],
        &gRoots[ 4], &gRoots[ 5], &gRoots[ 6], &gRoots[ 7],
        &gRoots[ 8], &gRoots[ 9], &gRoots[10], &gRoots[11],
        &gRoots[12], &gRoots[13], &gRoots[14], &gRoots[15],
        &gRoots[16], &gRoots[17], &gRoots[18], &gRoots[19],
        &gRoots[20], &gRoots[21], &gRoots[22], &gRoots[23],
        &gRoots[24], &gRoots[25], &gRoots[26], &gRoots[27],
        &gRoots[28], &gRoots[29], &gRoots[30], &gRoots[31],
        &gRoots[32], &gRoots[33], &gRoots[34], &gRoots[35],
        &gRoots[36], &gRoots[37], &gRoots[38], &gRoots[39],
        &gRoots[40], &gRoots[41], &gRoots[42], &gRoots[43],
        &gRoots[44], &gRoots[45], &gRoots[46], &gRoots[47],
        &gRoots[48], &gRoots[49], &gRoots[50], &gRoots[51],
        &gRoots[52], &gRoots[53], &gRoots[54], &gRoots[55],
        &gRoots[56], &gRoots[57], &gRoots[58], &gRoots[59],
        &gRoots[60], &gRoots[61], &gRoots[62], &gRoots[63],
      };
    };
  }
}
