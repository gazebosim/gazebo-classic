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

#include "gazebo/test/ServerFixture.hh"


namespace rhabarber::barbera::gazebo
{
    class bar
    {
    public:
        void print_something() const 
        {
            gzmsg << "something" << std::endl;
        }
    };
}


/////////////////////////////////////////////////
class Issue2896Test : public ::gazebo::ServerFixture
{
};

/////////////////////////////////////////////////
TEST_F(Issue2896Test, CompilationTest)
{
    rhabarber::barbera::gazebo::bar bar;
    // Suppress unused variable warning
    bar = bar;
}


/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
