/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <boost/filesystem.hpp>

#include "gazebo/common/Exception.hh"
#include "gazebo/util/LogRecord.hh"

/////////////////////////////////////////////////
/// \brief Stressing test to check start/stop threaded functionalty 
TEST(logrecord_stress, StartStopStressing)
{
  EXPECT_NO_THROW(
    gazebo::util::LogRecord *recorder = gazebo::util::LogRecord::Instance();
    recorder->Init("test");

    for (int j = 0; j < 10000; ++j)
    {
      recorder->Start();
      recorder->Stop();
    }
  );
}
