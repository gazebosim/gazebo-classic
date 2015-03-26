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

#include <gtest/gtest.h>

#include "gazebo/common/PID.hh"
#include "test/util.hh"

using namespace gazebo;

class PID : public gazebo::testing::AutoLogFixture { };

//////////////////////////////////////////////////
TEST_F(PID, ConstructorInitGetSet)
{
  // default constructor
  {
    common::PID pid;
    EXPECT_DOUBLE_EQ(0.0, pid.GetPGain());
    EXPECT_DOUBLE_EQ(0.0, pid.GetIGain());
    EXPECT_DOUBLE_EQ(0.0, pid.GetDGain());
    EXPECT_DOUBLE_EQ(0.0, pid.GetIMax());
    EXPECT_DOUBLE_EQ(0.0, pid.GetIMin());
    EXPECT_DOUBLE_EQ(0.0, pid.GetCmdMax());
    EXPECT_DOUBLE_EQ(0.0, pid.GetCmdMin());
  }

  const double p = 1.0;
  const double i = 2.0;
  const double d = 3.0;
  const double iMax =  4.0;
  const double iMin = -4.0;
  const double cmdMax =  5.0;
  const double cmdMin = -5.0;

  // Constructor with some parameters
  // Followed by Init with all parameters
  {
    common::PID pid(p, i, d);
    EXPECT_DOUBLE_EQ(p, pid.GetPGain());
    EXPECT_DOUBLE_EQ(i, pid.GetIGain());
    EXPECT_DOUBLE_EQ(d, pid.GetDGain());
    EXPECT_DOUBLE_EQ(0.0, pid.GetIMax());
    EXPECT_DOUBLE_EQ(0.0, pid.GetIMin());
    EXPECT_DOUBLE_EQ(0.0, pid.GetCmdMax());
    EXPECT_DOUBLE_EQ(0.0, pid.GetCmdMin());

    pid.Init(p, i, d, iMax, iMin, cmdMax, cmdMin);
    EXPECT_DOUBLE_EQ(p, pid.GetPGain());
    EXPECT_DOUBLE_EQ(i, pid.GetIGain());
    EXPECT_DOUBLE_EQ(d, pid.GetDGain());
    EXPECT_DOUBLE_EQ(iMax, pid.GetIMax());
    EXPECT_DOUBLE_EQ(iMin, pid.GetIMin());
    EXPECT_DOUBLE_EQ(cmdMax, pid.GetCmdMax());
    EXPECT_DOUBLE_EQ(cmdMin, pid.GetCmdMin());
  }

  // default constructor
  // followed by Set/Get of parameters
  {
    common::PID pid;
    pid.SetPGain(p);
    pid.SetIGain(i);
    pid.SetDGain(d);
    pid.SetIMax(iMax);
    pid.SetIMin(iMin);
    pid.SetCmdMax(cmdMax);
    pid.SetCmdMin(cmdMin);

    EXPECT_DOUBLE_EQ(p, pid.GetPGain());
    EXPECT_DOUBLE_EQ(i, pid.GetIGain());
    EXPECT_DOUBLE_EQ(d, pid.GetDGain());
    EXPECT_DOUBLE_EQ(iMax, pid.GetIMax());
    EXPECT_DOUBLE_EQ(iMin, pid.GetIMin());
    EXPECT_DOUBLE_EQ(cmdMax, pid.GetCmdMax());
    EXPECT_DOUBLE_EQ(cmdMin, pid.GetCmdMin());
  }
}

//////////////////////////////////////////////////
TEST_F(PID, Pcontrol)
{
  const double p = 1.0;
  // Proportional control
  {
    common::PID pid(p);
    const double error = 1.0;
    const double dt = 0.1;
    const double expectedCmd = -p * error;
    EXPECT_DOUBLE_EQ(expectedCmd, pid.Update(error, dt));
    EXPECT_DOUBLE_EQ(expectedCmd, pid.GetCmd());

    // Check GetErrors
    double pErr, iErr, dErr;
    pid.GetErrors(pErr, iErr, dErr);
    EXPECT_DOUBLE_EQ(error, pErr);
    EXPECT_DOUBLE_EQ(error*dt, iErr);
    EXPECT_DOUBLE_EQ(error/dt, dErr);

    // Update again with dt=0
    // Update should return 0.0
    // Other accessors should return same command and errors
    EXPECT_DOUBLE_EQ(0.0, pid.Update(error, 0.0));
    EXPECT_DOUBLE_EQ(expectedCmd, pid.GetCmd());
    pid.GetErrors(pErr, iErr, dErr);
    EXPECT_DOUBLE_EQ(error, pErr);
    EXPECT_DOUBLE_EQ(error*dt, iErr);
    EXPECT_DOUBLE_EQ(error/dt, dErr);
  }
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
