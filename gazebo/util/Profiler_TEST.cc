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

#include <gtest/gtest.h>

#include "gazebo/common/Time.hh"
#include "gazebo/util/Profiler.hh"
#include "test/util.hh"

using namespace gazebo;

class DiagnosticsTest : public gazebo::testing::AutoLogFixture { };

TEST_F(DiagnosticsTest, Diagnostics)
{
  util::ProfileManager *mgr = util::ProfileManager::Instance();
  EXPECT_TRUE(mgr != NULL);

  mgr->Reset();

  for(size_t ii = 0; ii < 100; ++ii) {
    mgr->Increment_Frame_Counter();
    mgr->Start_Profile("outer");
    {
      mgr->Start_Profile("inner");
      common::Time::MSleep(1);
      mgr->Stop_Profile();
    }
    common::Time::MSleep(2);
    mgr->Stop_Profile();
  }

  mgr->dumpAll();
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

