/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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

#include "common/Time.hh"

namespace gazebo
{
  namespace common
  {
    class PID
    {
      public: PID(double _p = 0.0, double _i = 0.0, double _d = 0.0,
                  double _imax = 0.0, double _imin = 0.0);
      public: virtual ~PID();

      public: double Update(double _error, common::Time _dt);

      private: double pErrLast;
      private: double pErr;
      private: double iErr;
      private: double dErr;
      private: double pGain;
      private: double iGain;
      private: double dGain;
      private: double iMax;
      private: double iMin;
    };
  }
}
