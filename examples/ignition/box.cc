#include <gazebo/math/Box.hh>
#include <gazebo/common/CommonTypes.hh>
#include <ignition/math/Box.hh>

/////////////////////////////////////////////////
// Example gazebo class that exposes an API using math::Box
// Release: Gazebo5
class SomeGazeboClass
{
  public: SomeGazeboClass()
          : box(gazebo::math::Vector3(0, 1, 2),
                gazebo::math::Vector3(3, 4, 5)) {}

  public: gazebo::math::Box Get() const
          {return this->box;}

  public: void Set(const gazebo::math::Box &_box)
          {this->box = _box;}

  private: gazebo::math::Box box;
};

/////////////////////////////////////////////////
// The above gazebo class that is transitioned to ignition math
// Release: Gazebo6
class SomeGazeboClassTransition
{
  public: SomeGazeboClassTransition()
          : box(0, 1, 2, 3, 4, 5) {}

  public: gazebo::math::Box Get() const
          GAZEBO_DEPRECATED(6.0)
          {return gazebo::math::Box(this->box);}

  public: ignition::math::Box GetIgn() const
          {return this->box;}

  public: void Set(const gazebo::math::Box &_box)
          GAZEBO_DEPRECATED(6.0)
          {this->box = _box.Ign();}

  public: void Set(const ignition::math::Box &_box)
          {this->box = _box;}

  private: ignition::math::Box box;
};

/////////////////////////////////////////////////
// The above gazebo class that is finalized to ignition math
// Release: Gazebo7
class SomeGazeboClassFinal
{
  public: SomeGazeboClassFinal()
          : box(0, 1, 2, 3, 4, 5) {}

  public: ignition::math::Box Get() const
          {return this->box;}

  public: void Set(const ignition::math::Box &_box)
          {this->box = _box;}

  private: ignition::math::Box box;
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
            cls.Set(gazebo::math::Box(gazebo::math::Vector3(6, 7, 8),
                                      gazebo::math::Vector3(9, 10, 11)));
            std::cout << cls.Get() << std::endl;
            box = cls.Get();
          }

  private: SomeGazeboClass cls;
  private: gazebo::math::Box box;
};

/////////////////////////////////////////////////
// Example class that uses the transitioned gazebo class
// Works with: Gazebo 6&7
class ExampleATransition
{
  public: ExampleATransition()
          {
            std::cout << cls.GetIgn() << std::endl;
            cls.Set(ignition::math::Box(6, 7, 8, 9, 10, 11));

            std::cout << cls.GetIgn() << std::endl;
            box = cls.GetIgn();
          }

  private: SomeGazeboClassTransition cls;
  private: ignition::math::Box box;
};

/////////////////////////////////////////////////
// Example class that uses the final gazebo class
// Release: Gazebo7
class ExampleAFinal
{
  public: ExampleAFinal()
          {
            std::cout << cls.Get() << std::endl;
            cls.Set(ignition::math::Box(6, 7, 8, 9, 10, 11));

            std::cout << cls.Get() << std::endl;
            box = cls.Get();
          }

  private: SomeGazeboClassFinal cls;
  private: ignition::math::Box box;
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
