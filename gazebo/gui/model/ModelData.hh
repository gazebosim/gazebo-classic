/*
 * Copyright 2013 Open Source Robotics Foundation
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
#ifndef _MODEL_DATA_HH_
#define _MODEL_DATA_HH_

#include <string>
#include <vector>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/physics/PhysicsTypes.hh"

#include "gazebo/gui/model/PartInspector.hh"

namespace gazebo
{
  namespace gui
  {
    /*/// \class VisualData VisualData.hh
    /// \brief Helper class to store Visual data
    class VisualData
    {
      /// \brief Get the name of the visual.
      /// \return Name of the visual
      public: std::string GetName() const;

      /// \brief Set the name of the visual.
      /// \param[in] _name Name of visual
      public: void SetName(const std::string &_name);

      /// \brief Get the pose of the visual.
      /// \return Pose of the visual
      public: math::Pose GetPose() const;

      /// \brief Set the pose of the visual.
      /// \param[in] _pose Pose of visual
      public: void SetPose(const math::Pose &_pose);

      /// \brief Get the geometry of the visual.
      /// \return Geometry of the visual
      public: std::string GetGeometry() const;

      /// \brief Set the geometry of the visual.
      /// \param[in] _geometry Geometry of visual
      public: void SetGeometry(const std::string &_geometry);

      /// \brief SDF representing the visual data.
      public: sdf::ElementPtr visualSDF;
    };*/


    /// \class CollisionData CollisionData.hh
    /// \brief Helper class to store collision data
    class CollisionData
    {
      /// \brief Constructor
      public: CollisionData();

      /// \brief Get the name of the collision.
      /// \return Name of collision.
      public: std::string GetName() const;

      /// \brief Set the name of the collision.
      /// \param[in] _name Name of collision.
      public: void SetName(const std::string &_name);

      /// \brief Get the pose of the collision.
      /// \return Pose of collision.
      public: math::Pose GetPose() const;

      /// \brief Set the pose of the collision.
      /// \param[in] _pose Pose of collision.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Get the laser retro of the collision.
      /// \return Laser retro.
      public: double GetLaserRetro() const;

      /// \brief Set the laser retro of collision.
      /// \param[in] _retro Laser retro.
      public: void SetLaserRetro(double _retro);

      /// \brief Get the maximum number of contacts of the collision.
      /// \return Maximum number of contacts.
      public: int GetMaxContacts() const;

      /// \brief Set the maximum number of contacts of collision.
      /// \param[in] _maxContacts Maximum number of contacts.
      public: void SetMaxContacts(int _maxContacts);

      /// \brief Get the surface restitution coefficient of the collision.
      /// \return Surface restitution coefficient.
      public: double GetSurfaceRestitutionCoeff() const;

      /// \brief Set the surface restitution coefficient of collision.
      /// \param[in] _coeff Surface restitution coefficient.
      public: void SetSurfaceRestitutionCoeff(double _coeff);

      /// \brief Name of collision.
      public: std::string name;

      /// \brief Pose of collision.
      public: math::Vector3 pose;

      /// \brief SDF representing the visual data.
      public: sdf::ElementPtr collisionSDF;
    };

    /// \class SensorData SensorData.hh
    /// \brief Helper class to store sensor data
    class SensorData
    {
      /// \brief Name of sensor.
      public: std::string name;

      /// \brief Type of sensor.
      public: std::string type;

      /// \brief Pose of sensor.
      public: math::Vector3 pose;

      /// \brief True to visualize sensor.
      public: bool visualize;

      /// \brief True to set sensor to be always on.
      public: bool alwaysOn;

      /// \brief Sensor topic name.
      public: std::string topicName;
    };

    /// \class PartData PartData.hh
    /// \brief Helper class to store part data
    class PartData : public QObject
    {
      Q_OBJECT

      /// \brief Constructor
      public: PartData();

      /// \brief Get the name of the part.
      /// \return Name of part.
      public: std::string GetName() const;

      /// \brief Set the name of the part.
      /// \param[in] _name Name of part.
      public: void SetName(const std::string &_name);

      /// \brief Get whether gravity is enabled for part.
      /// \return True if gravity is enabled.
      public: bool GetGravity() const;

      /// \brief Set the gravity of the part.
      /// \param[in] _gravity True to enable gravity on part.
      public: void SetGravity(bool _gravity);

      /// \brief Get if self collide is enabled for part.
      /// \return True if self collide is enabled.
      public: bool GetSelfCollide() const;

      /// \brief Set the self-collide property of the part.
      /// \param[in] _selfCollide True to enable self collision.
      public: void SetSelfCollide(bool _selfCollide);

      /// \brief Get whether the part is kinematic.
      /// \return True if the part is kinematic.
      public: bool GetKinematic() const;

      /// \brief Set the kinematic property of the part..
      /// \param[in] _kinematic True to enable kinematic mode.
      public: void SetKinematic(bool _kinematic);

      /// \brief Get the pose of the part.
      /// \return Pose of part.
      public: math::Pose GetPose() const;

      /// \brief Set the pose of the part.
      /// \param[in] _pose Pose of part.
      public: void SetPose(const math::Pose &_pose);

      /// \brief Get the mass of the part.
      /// \return Mass of part.
      public: double GetMass() const;

      /// \brief Set the mass of part.
      /// \param[in] _mass Mass of part.
      public: void SetMass(double _mass);

      /// \brief Get the inertial pose of the part.
      /// \return Inertial pose of part.
      public: math::Pose GetInertialPose() const;

      /// \brief Set the inertial pose of part.
      /// \param[in] _pose inertial pose of part.
      public: void SetInertialPose(const math::Pose &_pose);

      /// \brief Get the inertia ixx of the part.
      /// \return Inertia ixx of part.
      public: double GetInertiaIXX() const;

      /// \brief Set the inertia ixx element of part.
      /// \param[in] _ixx Inertia ixx of part.
      public: void SetInertiaIXX(double _ixx);

      /// \brief Get the inertia iyy of the part.
      /// \return Inertia iyy of part.
      public: double GetInertiaIYY() const;

      /// \brief Set the inertia iyy element of part.
      /// \param[in] _iyy Inertia iyy of part.
      public: void SetInertiaIYY(double _iyy);

      /// \brief Get the inertia izz of the part.
      /// \return Inertia izz of part.
      public: double GetInertiaIZZ() const;

      /// \brief Set the inertia izz element of part.
      /// \param[in] _izz Inertia izz of part.
      public: void SetInertiaIZZ(double _izz);

      /// \brief Get the inertia ixy of the part.
      /// \return Inertia ixy of part.
      public: double GetInertiaIXY() const;

      /// \brief Set the inertia ixy element of part.
      /// \param[in] _ixy Inertia ixy of part.
      public: void SetInertiaIXY(double _ixy);

      /// \brief Get the inertia ixz of the part.
      /// \return Inertia ixz of part.
      public: double GetInertiaIXZ() const;

      /// \brief Set the inertia ixz element of part.
      /// \param[in] _ixz Inertia ixz of part.
      public: void SetInertiaIXZ(double _ixz);

      /// \brief Get the inertia iyz of the part.
      /// \return Inertia iyz of part.
      public: double GetInertiaIYZ() const;

      /// \brief Set the inertia iyz element of part.
      /// \param[in] _iyz Inertia iyz of part.
      public: void SetInertiaIYZ(double _iyz);

      /// \brief SDF representing the part data.
      public: sdf::ElementPtr partSDF;

      /// \brief Visual representing this part.
      public: rendering::VisualPtr partVisual;

      /// \brief Visuals of the part.
      //public: std::map<std::string, VisualData *> visuals;
      public: std::vector<rendering::VisualPtr> visuals;

      /// \brief Collisions of the part.
      public: std::vector<CollisionData *> collisions;

      /// \brief Sensor data
      public: SensorData *sensorData;

      /// \brief Inspector for configuring part properties.
      public: PartInspector *inspector;

      /// \brief Qt Callback when part inspector configurations are to be
      /// applied.
      private slots: void OnApply();

      /// \brief Qt callback when a new visual is to be added.
      private slots: void OnAddVisual();

      /// \brief Qt callback when a visual is to be removed.
      /// \param[in] _name Name of visual.
      private slots: void OnRemoveVisual(const std::string &_name);
    };
  }
}

#endif
