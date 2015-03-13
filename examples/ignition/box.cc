#include <gazebo/math/Box.hh>
#include <gazebo/common/CommonTypes.hh>
#include <ignition/math/Box.hh>

/////////////////////////////////////////////////
// Example gazebo class that exposes an API using math::Box
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
// Example class that uses the original gazebo class
class ExampleA
{
  public: ExampleA()
          {
            std::cout << cls.Get() << std::endl;
            cls.Set(gazebo::math::Box(gazebo::math::Vector3(6, 7, 8),
                                      gazebo::math::Vector3(9, 10, 11)));
            std::cout << cls.Get() << std::endl;
          }

  private: SomeGazeboClass cls;
};

/////////////////////////////////////////////////
// Example class that uses the transitioned gazebo class
class ExampleB
{
  public: ExampleB()
          {
            std::cout << cls.Get() << std::endl;
            cls.Set(gazebo::math::Box(
                  gazebo::math::Vector3(6, 7, 8),
                  gazebo::math::Vector3(9, 10, 11)));
            std::cout << cls.Get() << std::endl;
            gzBox = cls.Get();

            cls.Set(ignition::math::Box(12, 13, 14, 15, 16, 17));
            std::cout << cls.Get() << std::endl;
            ignBox = cls.GetIgn();
          }

  private: SomeGazeboClassTransition cls;
  private: gazebo::math::Box gzBox;
  private: ignition::math::Box ignBox;
};


int main()
{
  std::cout << "Example A:\n";
  ExampleA a;

  std::cout << "Example B:\n";
  ExampleB b;
  return 0;
}
