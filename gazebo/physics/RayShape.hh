/*
 * Copyright 2011 Nate Koenig
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

#ifndef RAYSHAPE_HH
#define RAYSHAPE_HH

#include <float.h>
#include <string>

#include "physics/PhysicsTypes.hh"
#include "physics/Shape.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Base class for Ray collision geometry
    class RayShape : public Shape
    {
      /// \brief Constructor for a global ray
      public: RayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Constructor
      /// \param body Link the ray is attached to
      /// \param displayRays Indicates if the rays should be displayed when
      ///        rendering images
      public: RayShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~RayShape();

      /// \brief Set the ray based on starting and ending points relative to
      ///        the body
      /// \param posStart Start position, relative the body
      /// \param posEnd End position, relative to the body
      public: virtual void SetPoints(const math::Vector3 &posStart,
                                     const math::Vector3 &posEnd);


      /// \brief Get the relative starting and ending points
      /// \param posA Returns the starting point
      /// \param posB Returns the ending point
      public: virtual void GetRelativePoints(math::Vector3 &posA,
                                             math::Vector3 &posB);

      /// \brief Get the global starting and ending points
      /// \param posA Returns the starting point
      /// \param posB Returns the ending point
      public: virtual void GetGlobalPoints(math::Vector3 &posA,
                                           math::Vector3 &posB);

      /// \brief Set the length of the ray
      /// \param len Length of the array
      public: virtual void SetLength(double len);

      /// \brief Get the length of the ray
      public: double GetLength() const;

      /// \brief Update the ray collision
      public: virtual void Update() = 0;

      /// \brief Get the nearest intersection
      public: virtual void GetIntersection(double &_dist,
                                           std::string &_entity) = 0;

      /// \brief Set the retro-reflectivness detected by this ray
      public: void SetRetro(float retro);

      /// \brief Get the retro-reflectivness detected by this ray
      public: float GetRetro() const;

      /// \brief Set the fiducial id detected by this ray
      public: void SetFiducial(int fid);

      /// \brief Get the fiducial id detected by this ray
      public: int GetFiducial() const;

      /// \brief In the ray
      public: virtual void Init();

      public: void FillShapeMsg(msgs::Geometry &) {}
      public: virtual void ProcessMsg(const msgs::Geometry &) {}
      /// Contact information; this is filled out during collision
      /// detection.
      protected: double contactLen;
      protected: double contactRetro;
      protected: int contactFiducial;

      /// Start and end positions of the ray, relative to the body
      protected: math::Vector3 relativeStartPos;
      protected: math::Vector3 relativeEndPos;

      /// Start and end positions of the ray in global cs
      protected: math::Vector3 globalStartPos;
      protected: math::Vector3 globalEndPos;
    };
    /// \}
  }
}
#endif


