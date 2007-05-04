#ifndef BODY_HH
#define BODY_HH

#include <ode/ode.h>
#include <boost/any.hpp>
#include <list>
#include <vector>

#include "XMLConfig.hh"
#include "Entity.hh"
#include "Pose3d.hh"

class Geom;
namespace Ogre
{
  class SceneNode;
}

namespace gazebo
{

class Body : public Entity
{
  // Constructor
  public: Body(Entity *parent, dWorldID worldId);

  // Destructor
  public: virtual ~Body();

  //! Load the body based on an XMLConfig node
  /*!
   * \param node XMLConfigNode pointer
   * \return Nonzero on error
   */
  public: virtual int Load(XMLConfigNode *node);

  //! Initialize the body
  /*!
   * \return Non-zero on error
   */
  public: virtual int Init();

  //! Update the body
  /*!
   * \return Non-zero on error
   */
  public: virtual int Update();

  //! Attach a geom to this body
  /*!
   * \param geom Geometery to attach to this body
   */
  public: void AttachGeom( Geom *geom );

  //! Set the pose of the body
  /*!
   * \param pose New pose of the body
   */
  public: void SetPose(const Pose3d &pose);

  //! Return the pose of the body
  /*!
   * \return Pose of the body
   */
  public: Pose3d GetPose() const;

  //! Set the position of the body
  /*!
   * \param pos Vector position
   */
  public: void SetPosition(const Vector3 &pos);

  //! Set the rotation of the body
  /*!
   * \param rot Quaternion rotation
   */
  public: void SetRotation(const Quatern &rot);

  //! Return the position of the body
  /*!
   * \return Position vector
   */
  public: Vector3 GetPosition() const;

  //! Return the rotation
  /*!
   * \return Rotation quaternion
   */
  public: Quatern GetRotation() const;

  //! Return the ID of this body
  /*!
   * \return ODE body id
   */
  public: dBodyID GetId() const;

  // Set whether this body is enabled
  public: void SetEnabled(bool enable) const;

  //! Load a new geom helper function
  /*!
   * \param node XMLConfigNode used to load the geom
   * \return Non-zero on error
   */
  private: int LoadGeom(XMLConfigNode *node);

  // List of geometries attached to this body
  private: std::vector<Geom*> geoms;

  // ODE body handle
  private: dBodyID bodyId;

  // Mass properties of the object
  private: dMass mass;

  private: std::string name;

  private: bool isStatic;
};

}
#endif
