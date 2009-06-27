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
#include <map>
#include <vector>

#include "Entity.hh"
#include "Pose3d.hh"
#include "Param.hh"

namespace gazebo
{
  class Model;
  class Geom;
  class Sensor;
  class XMLConfigNode;
  class PhysicsEngine;

  /// \addtogroup gazebo_physics
  /// \brief The body class
  /// \{

  /// Body class
  class Body : public Entity
  {
    /// \brief Constructor
    public: Body(Entity *parent);

    /// \brief Destructor
    public: virtual ~Body();

    /// \brief Load the body based on an XMLConfig node
    /// \param node XMLConfigNode pointer
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save the body based on our XMLConfig node
    public: virtual void Save(std::string &prefix, std::ostream &stream);

    /// \brief Initialize the body
    public: virtual void Init();

    /// \brief Finalize the body
    public: void Fini();

    /// \brief Update the body
    public: virtual void Update();

    /// \brief Attach a geom to this body
    /// \param geom Geometery to attach to this body
    public: void AttachGeom( Geom *geom );

    /// \brief Set the pose of the body
    /// \param pose New pose of the body
    public: void SetPose(const Pose3d &pose);

    /// \brief Return the pose of the body
    /// \return Pose of the body
    public: virtual Pose3d GetPose() const;

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

    /// \brief Return the velocity of the body
    /// \return Velocity vector
    public: Vector3 GetPositionRate() const;

    /// \brief Return the rotation rates
    /// \return Rotation Rate quaternion
    public: Quatern GetRotationRate() const;

    /// \brief Return the rotation rates
    /// \return Rotation Rate Euler Angles RPY
    public: Vector3 GetEulerRate() const;

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

    /// \brief Set the friction mode of the body
    public: void SetFrictionMode( const bool &v );

    /// \brief Set the collide mode of the body
    public: void SetCollideMode( const std::string &m );

    /// \brief Get Self-Collision Flag, if this is true, this body will collide
    //         with other bodies even if they share the same parent.
    public: bool GetSelfCollide();

    /// \brief Set the laser fiducial integer id
    public: void SetLaserFiducialId(int id);

   /// \brief Set the laser retro reflectiveness
    public: void SetLaserRetro(float retro);

    /// \brief Set the linear velocity of the body
    public: void SetLinearVel(const Vector3 &vel);

    /// \brief Get the linear velocity of the body
    public: Vector3 GetLinearVel() const;

    /// \brief Set the angular velocity of the body
    public: void SetAngularVel(const Vector3 &vel);

    /// \brief Get the angular velocity of the body
    public: Vector3 GetAngularVel() const;

    /// \brief Set the linear acceleration of the body
    public: void SetLinearAccel(const Vector3 &accel);

    /// \brief Get the linear acceleration of the body
    public: Vector3 GetLinearAccel() const;

    /// \brief Set the angular acceleration of the body
    public: void SetAngularAccel(const Vector3 &accel);

    /// \brief Get the angular acceleration of the body
    public: Vector3 GetAngularAccel() const;

    /// \brief Set the force applied to the body
    public: void SetForce(const Vector3 &force);

    /// \brief Get the force applied to the body
    public: Vector3 GetForce() const;

    /// \brief Set the torque applied to the body
    public: void SetTorque(const Vector3 &force);

    /// \brief Get the torque applied to the body
    public: Vector3 GetTorque() const;

    /// \brief Get the vector of all geoms
    public: const std::map<std::string, Geom*> *GetGeoms() const;

    /// \brief Get a geom by name
    public: Geom *GetGeom(const std::string &name) const;

    /// \brief Get the model that this body belongs to
    public: Model *GetModel() const;

    /// \brief Get the mass of the body
    public: float GetMass() const { return mass.mass; }

    /// \brief Get the list of interfaces e.g "pioneer2dx_model1::laser::laser_iface0->laser"
    public: void GetInterfaceNames(std::vector<std::string>& list) const;

    /// \brief Get a sensor by name
    public: Sensor *GetSensor( const std::string &name ) const;

    /// Load a new geom helper function
    /// \param node XMLConfigNode used to load the geom
    private: void LoadGeom(XMLConfigNode *node);

    /// Load a new sensor
    /// \param node XMLConfigNode used to load the geom
    private: void LoadSensor(XMLConfigNode *node);

    /// \brief Load a renderable
    private: void LoadVisual(XMLConfigNode *node);

    /// \brief Update the pose of the body
    private: void UpdatePose();

    /// \brief Set transparency for all child geometries
    public: void SetTransparency(float t);

    /// \brief Returns list of sensors
    public:  std::vector< Sensor* > &GetSensors();

    /// \brief  Get the size of the body
    public: void GetBoundingBox(Vector3 &min, Vector3 &max) const;

    /// List of geometries attached to this body
    private: std::map< std::string, Geom* > geoms;

    /// List of attached sensors
    private: std::vector< Sensor* > sensors;

    /// ODE body handle
    private: dBodyID bodyId;

    /// Mass properties of the object
    private: dMass mass;

    private: bool isStatic;

    private: Pose3d comPose;
    private: Pose3d staticPose;
    private: Pose3d pose;

    private: ParamT<Vector3> *xyzP;
    private: ParamT<Quatern> *rpyP;

    private: ParamT<double> *dampingFactorP;

    private: PhysicsEngine *physicsEngine;

    private: ParamT<bool> *turnGravityOffP;
    private: ParamT<bool> *selfCollideP;

    private: OgreVisual *cgVisual;

    private: Vector3 linearAccel;
    private: Vector3 angularAccel;

    ///  User specified Mass Matrix
    protected: ParamT<bool> *customMassMatrixP;
    protected: ParamT<double> *cxP ;
    protected: ParamT<double> *cyP ;
    protected: ParamT<double> *czP ;
    protected: ParamT<double> *bodyMassP;
    protected: ParamT<double> *ixxP;
    protected: ParamT<double> *iyyP;
    protected: ParamT<double> *izzP;
    protected: ParamT<double> *ixyP;
    protected: ParamT<double> *ixzP;
    protected: ParamT<double> *iyzP;
    protected: bool customMassMatrix;
    protected: double cx,cy,cz,bodyMass,ixx,iyy,izz,ixy,ixz,iyz;
  };

  /// \}
}

#endif
