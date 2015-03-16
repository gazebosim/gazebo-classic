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
// The above gazebo class that is transitioned to ignition math
// Release: Gazebo6
class SomeGazeboClassTransition
{
  public: SomeGazeboClassTransition()
          : angle(0.1) {}

  public: gazebo::math::Angle Get() const
          GAZEBO_DEPRECATED(6.0)
          {return gazebo::math::Angle(this->angle);}

  public: ignition::math::Angle GetIgn() const
          {return this->angle;}

  public: void Set(const gazebo::math::Angle &_angle)
          GAZEBO_DEPRECATED(6.0)
          {this->angle = _angle.Ign();}

  public: void Set(const ignition::math::Angle &_angle)
          {this->angle = _angle;}

  private: ignition::math::Angle angle;
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
// Example class that uses the transitioned gazebo class
// Works with: Gazebo 6&7
class ExampleATransition
{
  public: ExampleATransition()
          {
            std::cout << cls.GetIgn() << std::endl;
            cls.Set(ignition::math::Angle(0.1));

            std::cout << cls.GetIgn() << std::endl;
            angle = cls.GetIgn();
          }

  private: SomeGazeboClassTransition cls;
  private: ignition::math::Angle angle;
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

  std::cout << "Example A Transition:\n";
  ExampleATransition b;

  std::cout << "Example A Final:\n";
  ExampleAFinal c;

  return 0;
}
