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

#include <boost/filesystem.hpp>
#include "gazebo/common/Console.hh"

using namespace gazebo;

namespace gazebo 
{
    namespace testing 
    {
        class AutoLogFixture : public ::testing::Test
        {
          protected:
            /// \brief Setup the test fixture. This gets called by gtest.
            virtual void SetUp() 
            {
              ASSERT_TRUE(getenv("HOME") != NULL);
              
              // We need to create the log directory if needed.
              boost::filesystem::path log_path(getenv("HOME"));
              log_path = log_path / ".gazebo";
              if (!boost::filesystem::exists(log_path))
                boost::filesystem::create_directories(log_path);

              const ::testing::TestInfo* const test_info =
                ::testing::UnitTest::GetInstance()->current_test_info();

              std::string test_name       = test_info->name();
              std::string test_case_name  = test_info->test_case_name();
              log_filename_  = test_case_name + "_" + test_name + ".log";
              log_directory_ = log_path.string();
              
              // Initialize Console
              gzLogInit(log_filename_);
              gazebo::common::Console::SetQuiet(false);
            }

            /// \brief Return a string with the full log file path
            std::string get_full_log_path() const
            {
              return log_directory_ + "/" + log_filename_;
            }

            /// \brief Return a string with all the log content loaded from the disk
            std::string get_log_content() const
            {
              // Open the log file, and read back the string
              std::ifstream ifs(get_full_log_path().c_str(), std::ios::in);
              std::string loggedString;

              while (!ifs.eof())
              {
                std::string line;
                std::getline(ifs, line);
                loggedString += line;
              }

              return loggedString;
            }
 
            /// \brief String with the full path of the logfile
            std::string log_filename_;
            /// \brief String with the full path to log directory
            std::string log_directory_;
        };
    }
}

#endif
