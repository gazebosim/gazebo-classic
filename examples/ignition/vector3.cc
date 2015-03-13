#include <gazebo/math/Vector3.hh>
#include <ignition/math/Vector3.hh>

int main()
{
  // Construction
  gazebo::math::Vector3 gzVec(1, 2, 3);
  ignition::math::Vector3d ignVec(1, 2, 3);

  // Set
  gzVec.x = 2.5;
  ignVec.X() = 3.5;
  ignVec.X(4.5);

  // Get
  std::cout << gzVec.x << std::endl;
  std::cout << ignVec.X() << std::endl;

  // Convert from ignition to gazebo
  gzVec = ignVec;

  // Convert gazebo to igntion
  ignVec = gzVec.Ign();

  return 0;
}
