/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
/* Desc: A timer class
 * Author: Nate Koenig
 * Date: 22 Nov 2009
 */

#ifndef _TIMER_HH_
#define _TIMER_HH_

#include "gazebo/common/Console.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common
    /// \{

    /// \class Timer Timer.hh common/common.hh
    /// \brief A timer class, used to time things in real world walltime
    class GZ_COMMON_VISIBLE Timer
    {
      /// \brief Constructor
      public: Timer();

      /// \brief Destructor
      public: virtual ~Timer();

      /// \brief Start the timer
      public: virtual void Start();

      /// \brief Stop the timer
      public: virtual void Stop();

      /// \brief Returns true if the timer is running.
      /// \return Tue if the timer has been started and not stopped.
      public: bool GetRunning() const;

      /// \brief Get the elapsed time
      /// \return The time
      public: Time GetElapsed() const;

      /// \brief Reset the timer
      public: void Reset();

      /// \brief Stream operator friendly
      public: friend std::ostream &operator<<(std::ostream &out,
                                              const gazebo::common::Timer &t)
              {
                out << t.GetElapsed();
                return out;
              }

      /// \brief True if a reset is needed.
      private: bool reset;

      /// \brief True if the timer is running.
      private: bool running;

      /// \brief The time of the last call to Start
      private: Time start;

      /// \brief The time when Stop was called.
      private: Time stop;
    };
    /// \}
  }
}
#endif


