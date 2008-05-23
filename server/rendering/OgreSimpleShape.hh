#ifndef OGRESIMPLESHAPE_HH
#define OGRESIMPLESHAPE_HH

#include <Ogre.h>

#include "Vector3.hh"

namespace gazebo
{
/// \addtogroup gazebo_rendering
/// \{

/// \brief Class used to render simple shapes
class OgreSimpleShape
{
  /// \brief Constructor
  private: OgreSimpleShape();

  /// \brief Destructor
  private: ~OgreSimpleShape();

  /// \brief Create a sphere mesh
  public: static void CreateSphere(const std::string &name, float radius, int rings, int segments);

  /// \brief Create a Box mesh
  public: static void CreateBox(const std::string &name, const Vector3 &sides);

  /// \brief Create a cylinder mesh
  public: static void CreateCylinder(const std::string &name, float radius, float height, int rings, int segments);

};

/// \}
}

#endif
