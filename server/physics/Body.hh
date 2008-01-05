/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#ifndef BODY_HH
#define BODY_HH

#include <ode/ode.h>
#include <boost/any.hpp>
#include <list>
#include <vector>

#include "UpdateParams.hh"
#include "XMLConfig.hh"
#include "Entity.hh"
#include "Pose3d.hh"

namespace gazebo
{
  class Geom;
  class Sensor;

/// \addtogroup gazebo_physics
/// \brief The body class
/// \{

/// Body class
class Body : public Entity
{
  /// \brief Constructor
  public: Body(Entity *parent, dWorldID worldId);

  /// \brief Destructor
  public: virtual ~Body();

  /// \brief Load the body based on an XMLConfig node
  /// \param node XMLConfigNode pointer
  public: virtual void Load(XMLConfigNode *node);

  /// \brief Save the body based on our XMLConfig node
  public: virtual void Save();
  
  /// \brief Initialize the body
  public: virtual void Init();

  /// \brief Finalize the body
  public: void Fini();

  /// \brief Update the body
  public: virtual void Update(UpdateParams &params);

  /// \brief Attach a geom to this body
  /// \param geom Geometery to attach to this body
  public: void AttachGeom( Geom *geom );

  /// \brief Set the pose of the body
  /// \param pose New pose of the body
  public: void SetPose(const Pose3d &pose);

  /// \brief Return the pose of the body
  /// \return Pose of the body
  public: Pose3d GetPose() const;

  /// \brief Set the position of the body
  /// \param pos Vector position
  public: void SetPosition(const Vector3 &pos);

  /// \brief Set the rotation of the body
  /// \param rot Quaternion rotation
  public: void SetRotation(const Quatern &rot);

  /// \brief Return the position of the body
  /// \return Position vector
  public: Vector3 GetPosition() const;

  /// \brief Return the rotation
  /// \return Rotation quaternion
  public: Quatern GetRotation() const;

  /// \brief Return the ID of this body
  /// \return ODE body id
  public: dBodyID GetId() const;

  /// \brief Set whether this body is enabled
  public: void SetEnabled(bool enable) const;

  /// \brief Update the center of mass
  public: void UpdateCoM();

  /// \brief Get the Center of Mass pose
  public: const Pose3d &GetCoMPose() const;

  /// \brief Set whether gravity affects this body
  public: void SetGravityMode(bool mode);

  /// \brief Set the linear velocity of the body
  public: void SetLinearVel(const Vector3 &vel);

  /// \brief Get the linear velocity of the body
  public: Vector3 GetLinearVel() const;

  /// \brief Set the angular velocity of the body
  public: void SetAngularVel(const Vector3 &vel);

  /// \brief Get the angular velocity of the body
  public: Vector3 GetAngularVel() const;

  /// \brief Set the force applied to the body
  public: void SetForce(const Vector3 &force);

  /// \brief Get the force applied to the body
  public: Vector3 GetForce() const;

  /// \brief Set the torque applied to the body
  public: void SetTorque(const Vector3 &force);

  /// \brief Get the torque applied to the body
  public: Vector3 GetTorque() const;

  /// Load a new geom helper function
  /// \param node XMLConfigNode used to load the geom
  /// \return Non-zero on error
  private: int LoadGeom(XMLConfigNode *node);

  /// Load a new sensor
  /// \param node XMLConfigNode used to load the geom
  private: void LoadSensor(XMLConfigNode *node);

  /// \brief Load a renderable
  private: void LoadVisual(XMLConfigNode *node);

  /// List of geometries attached to this body
  private: std::vector< Geom* > geoms;

  /// List of attached sensors
  private: std::vector< Sensor* > sensors;
  
  ///our XML DATA
  private: XMLConfigNode *xmlNode;
  
  /// ODE body handle
  private: dBodyID bodyId;

  /// Mass properties of the object
  private: dMass mass;

  private: bool isStatic;

  private: Pose3d comPose;
  private: Pose3d staticPose;
};

/// \}
}
#endif
