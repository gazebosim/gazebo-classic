/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_SURFACEPARAMS_HH_
#define _GAZEBO_SURFACEPARAMS_HH_

#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \class FrictionPyramid SurfaceParams.hh physics/physics.hh
    /// \brief Parameters used for friction pyramid model.
    class GZ_PHYSICS_VISIBLE FrictionPyramid
    {
      /// \brief Constructor.
      public: FrictionPyramid();

      /// \brief Destructor.
      public: virtual ~FrictionPyramid();

      /// \brief Get the friction coefficient in the primary direction.
      /// \return Friction coefficient in primary direction.
      public: double MuPrimary() const;

      /// \brief Get the friction coefficient in the secondary direction.
      /// \return Friction coefficient in secondary direction.
      public: double MuSecondary() const;

      /// \brief Get the torsional friction coefficient.
      /// \return Torsional friction coefficient.
      public: double MuTorsion() const;

      /// \brief Get the torsional patch radius.
      /// \return Torsional patch radius.
      public: double PatchRadius() const;

      /// \brief Get the torsional surface radius.
      /// \return Torsional surface radius.
      public: double SurfaceRadius() const;

      /// \brief Get the torsional "use patch radius" flag.
      /// \return Torsional "use patch radius" flag.
      public: bool UsePatchRadius() const;

      /// \brief Set the friction coefficient in the primary direction.
      /// \param[in] _mu Friction coefficient.
      public: void SetMuPrimary(double _mu);

      /// \brief Set the friction coefficient in the secondary direction.
      /// \param[in] _mu Friction coefficient.
      public: void SetMuSecondary(double _mu);

      /// \brief Set the torsional friction coefficient.
      /// \param[in] _mu Torsional friction coefficient.
      public: void SetMuTorsion(const double _mu);

      /// \brief Set the torsional patch radius.
      /// \param[in] _radius Torsional patch radius.
      public: void SetPatchRadius(const double _radius);

      /// \brief Set the torsional surface radius.
      /// \param[in] _radius Torsional surface radius.
      public: void SetSurfaceRadius(const double _radius);

      /// \brief Set whether to use the surface radius.
      /// \param[in] _use True to use the surface radius.
      public: void SetUsePatchRadius(const bool _use);

      /// \brief Get the Poisson's ratio
      /// \return Poisson's ratio
      public: double PoissonsRatio() const;

      /// \brief Set the Poisson's ratio
      /// \param[in] _ratio Poisson's ratio
      public: void SetPoissonsRatio(const double _ratio);

      /// \brief Get the elastic modulus
      /// \return elastic modulus
      public: double ElasticModulus() const;

      /// \brief Set the elastic modulus
      /// \param[in] _modulus elastic modulus to set to
      public: void SetElasticModulus(const double _modulus);

      /// \brief Get the friction coefficient in a single direction.
      /// \param[in] _index Index of friction direction, 0 for primary,
      /// 1 for secondary direction.
      /// \return Friction coefficient, or negative value if invalid
      /// _index is supplied.
      private: double Mu(const unsigned int _index) const;

      /// \brief Set the friction coefficient in a single direction.
      /// If a negative value is supplied, use an astronomically high
      /// value instead.
      /// \param[in] _index Index of friction direction, 0 for primary,
      /// 1 for secondary direction.
      /// \param[in] _mu Friction coefficient.
      private: void SetMu(unsigned int _index, double _mu);

      /// \brief Vector for specifying the primary friction direction,
      /// relative to the parent collision frame. The component of this
      /// vector that is orthogonal to the surface normal will be set
      /// as the primary friction direction.
      /// If undefined, a vector consstrained to be perpendicular
      /// to the contact normal in the global y-z plane is used.
      /// \sa http://www.ode.org/ode-latest-userguide.html#sec_7_3_7
      public: math::Vector3 direction1;

      /// \brief Array of dry friction coefficients. mu[0] is in the
      /// primary direction as defined by the friction pyramid.
      /// mu[1] is in the second direction.
      /// mu[2] is in the torsional direction.
      private: double mu[3];

      /// \brief Radius of the contact patch used to calculate torsional
      /// friction. The same patch is used by all contact points in this
      /// surface.
      /// The patch is equal to the square root of the surface radius times the
      /// contact depth: a = srqt(R * d).
      private: double patchRadius;

      /// \brief Radius of the surface to be used to calculate torsional
      /// friction. The same radius is used for all contacts points in this
      /// surface.
      private: double surfaceRadius;

      /// \brief Flag for choosing the method for computing the contact
      /// patch radius in torsional friction.
      /// True to use the constant patch radius parameter. False to use the
      /// surface radius together with contact depth.
      private: bool usePatchRadius;

      /// \brief Poisson's Ratio
      private: double poissonsRatio;

      /// \brief Elastic modulus.
      private: double elasticModulus;
    };

    /// \class SurfaceParams SurfaceParams.hh physics/physics.hh
    /// \brief SurfaceParams defines various Surface contact parameters.
    /// These parameters defines the properties of a
    /// physics::Contact constraint.
    class GZ_PHYSICS_VISIBLE SurfaceParams
    {
      /// \brief Constructor.
      public: SurfaceParams();

      /// \brief Destructor.
      public: virtual ~SurfaceParams();

      /// \brief Load the contact params.
      /// \param[in] _sdf SDF values to load from.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Fill in a surface message.
      /// \param[in] _msg Message to fill with this object's values.
      public: virtual void FillMsg(msgs::Surface &_msg);

      /// \brief Process a surface message.
      /// \param[in] _msg Message to read values from.
      public: virtual void ProcessMsg(const msgs::Surface &_msg);

      /// \brief Get access to FrictionPyramid data, if available.
      /// \return Pointer to FrictionPyramid data or NULL if class does
      /// not use FrictionPyramid data.
      public: virtual FrictionPyramidPtr FrictionPyramid() const;

      /// \brief Allow collision checking without generating a contact joint.
      public: bool collideWithoutContact;

      /// \brief Custom collision filtering used when collideWithoutContact is
      /// true.
      public: unsigned int collideWithoutContactBitmask;

      /// \brief Custom collision filtering. Will override
      /// collideWithoutContact.
      public: unsigned int collideBitmask;
    };
    /// \}
  }
}
#endif
