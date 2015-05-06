/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include <iostream>
#include <boost/shared_ptr.hpp>

#include "ServerFixture.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/common.hh"

using namespace std;
using namespace gazebo;

class ImageConvertStressTest : public ServerFixture
{
  /////////////////////////////////////////////////
  public: double virtMemory()
  {
    double resident, share;
    GetMemInfo(resident, share);
    return resident + share;
  }

  public: static void delete_many(unsigned char* ptr)
  {
    delete[] ptr;
  }
};

/////////////////////////////////////////////////
TEST_F(ImageConvertStressTest, ManyConversions)
{
  boost::shared_ptr<unsigned char> u(new unsigned char[400*400*3], delete_many);
  double memBefore = virtMemory();

  for (int i = 0; i < 1000; i++)
  {
    common::Image image;
    msgs::Image msg;

    image.SetFromData(u.get(), 400, 400, common::Image::RGB_INT8);

    msgs::Set(&msg, image);
  }

  double memAfter = virtMemory();

  // Without the fix in pull request #1057, the difference is over 470000
  EXPECT_LE(memAfter - memBefore, 2000);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
