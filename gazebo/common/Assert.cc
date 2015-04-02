/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <inttypes.h>
#include "Assert.hh"
#include "gazebo/common/Exception.hh"

/*
 * When using the flag BOOST_ENABLE_ASSERT_HANDLER, the boost functions
 * assert_failed and assertion_failed_msg function need to be defined. Gazebo
 * behaviour is to thrown a gazebo::common::AssertionInternalError.
 */
namespace boost
{
  void assertion_failed(char const * expr, char const * function,
                        char const * file, int64_t line)
  {
    throw gazebo::common::AssertionInternalError(file, line, expr, function);
  }


  void assertion_failed_msg(char const * expr, char const * msg,
                            char const * function, char const * file,
                            int64_t line)
  {
    throw gazebo::common::AssertionInternalError(file, line, expr,
                                                 function, msg);
  }
}
