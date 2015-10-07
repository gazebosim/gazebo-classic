/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: A ray
 * Author: Nate Koenig
 * Date: 14 Oct 2009
*/

#ifndef _RAYSHAPE_HH_
#define _RAYSHAPE_HH_

#include <string>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class RayShape RayShape.hh physics/physics.hh
    /// \brief Base class for Ray collision geometry
    class GZ_PHYSICS_VISIBLE RayShape : public Shape
    {
      /// \brief Constructor for a global ray.
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit RayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor.
      /// \param[in] _parent Collision parent of the shape.
      public: explicit RayShape(CollisionPtr _parent);

      /// \brief Destructor.
      public: virtual ~RayShape();

      /// \brief Set the ray based on starting and ending points relative to
      /// the body.
      /// \param[in] _posStart Start position, relative the body.
      /// \param[in] _posEnd End position, relative to the body.
      public: virtual void SetPoints(const math::Vector3 &_posStart,
                                     const math::Vector3 &_posEnd);


      /// \brief Get the relative starting and ending points.
      /// \param[in] _posA Returns the starting point.
      /// \param[in] _posB Returns the ending point.
      public: virtual void GetRelativePoints(math::Vector3 &_posA,
                                             math::Vector3 &_posB);

      /// \brief Get the global starting and ending points.
      /// \param[out] _posA Returns the starting point.
      /// \param[out] _posB Returns the ending point.
      public: virtual void GetGlobalPoints(math::Vector3 &_posA,
                                           math::Vector3 &_posB);

      /// \brief Set the length of the ray.
      /// \param[in] _len Length of the array.
      public: virtual void SetLength(double _len);

      /// \brief Get the length of the ray.
      /// \return The ray length.
      public: double GetLength() const;

      /// \brief Set the scale of the ray
      public: virtual void SetScale(const math::Vector3 &_scale);

      /// \brief Update the ray collision.
      public: virtual void Update() = 0;

      /// \brief Get the nearest intersection.
      /// \param[out] _dist Distance to the intersection.
      /// \param[out] _entity Name of the entity the ray intersected with.
      public: virtual void GetIntersection(double &_dist,
                                           std::string &_entity) = 0;

      /// \brief Set the retro-reflectivness detected by this ray.
      /// \param[in] _retro Retro reflectance value.
      public: void SetRetro(float _retro);

      /// \brief Get the retro-reflectivness detected by this ray.
      /// \return Retro reflectance value.
      public: float GetRetro() const;

      /// \brief Set the fiducial id detected by this ray.
      /// \param[in] _fid Fiducial id detected by this ray.
      public: void SetFiducial(int _fid);

      /// \brief Get the fiducial id detected by this ray.
      /// \return Fiducial id detected.
      public: int GetFiducial() const;

      /// \brief In the ray.
      public: virtual void Init();

      /// \brief Fill a message with data from this object.
      /// \param[out] _msg Message to fill.
      /// \TODO Implement this function.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \brief Update this shape from a message.
      /// \param[in] _msg Message to update from.
      /// \TODO Implement this function.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;

      // Contact information; this is filled out during collision
      // detection.
      /// \brief Length of the ray.
      protected: double contactLen;
      /// \brief Retro reflectance value
      protected: double contactRetro;
      /// \brief Fiducial ID value.
      protected: int contactFiducial;

      /// \brief Start position of the ray, relative to the body
      protected: math::Vector3 relativeStartPos;
      /// \brief End position of the ray, relative to the body
      protected: math::Vector3 relativeEndPos;

      /// \brief Start position of the ray in global cs
      protected: math::Vector3 globalStartPos;

      /// \brief End position of the ray in global cs
      protected: math::Vector3 globalEndPos;
    };
    /// \}
  }
}
#endif
