/*
 * copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _REST_EXCEPTION_HH_
#define _REST_EXCEPTION_HH_

#include <stdexcept>

namespace gazebo
{
  // Basic exception class that inherits from the standard runtime error.
  class RestException : public std::runtime_error
  {
    public: RestException(const char *_m):std::runtime_error(_m) {}
  };
}

#endif

