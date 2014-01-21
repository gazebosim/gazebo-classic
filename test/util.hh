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
#ifndef _GAZEBO_TEST_UTIL_HH_
#define _GAZEBO_TEST_UTIL_HH_

#include <gtest/gtest.h>

#include "gazebo/common/Console.hh"

using namespace gazebo;

namespace gazebo 
{
    namespace testing 
    {
        class AutoLogFixture : public ::testing::Test
        {
          protected:
            virtual void SetUp() 
            {
              const ::testing::TestInfo* const test_info =
                ::testing::UnitTest::GetInstance()->current_test_info();

              std::string test_name       = test_info->name();
              std::string test_case_name  = test_info->test_case_name();
              
              // Initialize Console
              gzLogInit(test_case_name + "_" + test_name + ".log");
              gazebo::common::Console::SetQuiet(false);
            }
        };
    }
}

#endif
