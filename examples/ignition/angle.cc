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
#include <gazebo/math/Angle.hh>
#include <gazebo/common/CommonTypes.hh>
#include <ignition/math/Angle.hh>


/////////////////////////////////////////////////
// Example gazebo class that exposes an API using math::Angle
// Release: Gazebo5
class SomeGazeboClass
{
  public: SomeGazeboClass()
          : angle(0.1) {}

  public: gazebo::math::Angle Get() const
          {return this->angle;}

  public: void Set(const gazebo::math::Angle &_angle)
          {this->angle = _angle;}

  private: gazebo::math::Angle angle;
};

/////////////////////////////////////////////////
// The above gazebo class that is finalized to ignition math
// Release: Gazebo7
class SomeGazeboClassFinal
{
  public: SomeGazeboClassFinal()
          : angle(0.1) {}

  public: ignition::math::Angle Get() const
          {return this->angle;}

  public: void Set(const ignition::math::Angle &_angle)
          {this->angle = _angle;}

  private: ignition::math::Angle angle;
};


/////////////////////////////////////////////////
/////////////////////////////////////////////////
// Classes that use the above Gazebo examples
/////////////////////////////////////////////////

/////////////////////////////////////////////////
// Example class that uses the original gazebo class
// Works with: Gazebo 5&6
class ExampleA
{
  public: ExampleA()
          {
            std::cout << cls.Get() << std::endl;
            cls.Set(gazebo::math::Angle(0.1));
            std::cout << cls.Get() << std::endl;
            angle = cls.Get();
          }

  private: SomeGazeboClass cls;
  private: gazebo::math::Angle angle;
};

/////////////////////////////////////////////////
// Example class that uses the final gazebo class
// Release: Gazebo7
class ExampleAFinal
{
  public: ExampleAFinal()
          {
            std::cout << cls.Get() << std::endl;
            cls.Set(ignition::math::Angle(0.1));

            std::cout << cls.Get() << std::endl;
            angle = cls.Get();
          }

  private: SomeGazeboClassFinal cls;
  private: ignition::math::Angle angle;
};

int main()
{
  std::cout << "Example A:\n";
  ExampleA a;

  std::cout << "Example A Final:\n";
  ExampleAFinal c;

  return 0;
}
