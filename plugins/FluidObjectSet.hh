/* Desc: Object Set class
 * Author: Andrei Haidu
 * Date: 11 Jul. 2013
 */

#ifndef OBJECT_SET_HH
#define OBJECT_SET_HH

#include "vector_types.h"
#include "FluidQuaternion.h"

#include "gazebo/util/system.hh"

/// \brief namespace for fluids
namespace fluidix
{
    /// \class ObjectSet ObjectSet.hh
    /// \brief ObjectSet class
    class GAZEBO_VISIBLE ObjectSet
    {
        /// \brief Constructor
        public: ObjectSet();

        /// \brief Destructor
        public: virtual ~ObjectSet();

        /// \brief Load
        public: virtual void Load();

        /// \brief Finalize
        public: virtual void Fini();

        /// \brief Initialize
        public: virtual void Init();

        /// \brief Reset
        public: virtual void Reset();

        /// \brief Update
        public: void Update();

        /// \brief Set particle set ID
        public: void SetParticleSetId(int _id);

        /// \brief Get particle set ID
        public: int GetParticleSetId();

        /// \brief Set Link set ID
        public: void SetLinkSetId(int _linkSetId);

        /// \brief Return ID of the Link set
        public: int GetLinkId();

        /// \brief Set ID of the Particle and Link Set
        public: void SetParticleAndLinkSetId(int _particleSetId, int _linkSetId);

        /// \brief Set ID of the Particle and Link Set
        public: void SetParticleAndLinkSetId(int2 _particleLinkSetId);

        /// \brief Set object restitution coeficient
        public: void SetRestitutionCoef(float _coef);

        /// \brief Get object restitution coeficient
        public: float GetRestitutionCoef();

        /// \brief Set object collision force sum
        public: void SetCollisionForceSum(float4 _sum_coll_force);

        /// \brief Get object collision force sum
        public: float4 GetCollisionForceSum();

        /// \brief Set object collision force sum position
        public: void SetCollisionForcePosSum(float4 _sum_coll_force_pos);

        /// \brief Get object collision force sum position
        public: float4 GetCollisionForcePosSum();

        /// \brief Set object world position
        public: void SetWorldPosition(float4 _worldPos);

        /// \brief Set object world position
        public: void SetWorldPosition(float _x, float _y, float _z);

        /// \brief Set object world position
        public: void SetWorldPosition(float3 _worldPos3);

        /// \brief Get object world position
        public: float4 GetWorldPosition();

        /// \brief Set object world orientation
        public: void SetWorldOrientation(Quaternion _quat);

        /// \brief Set object world orientation
        public: Quaternion GetWorldOrientation();


        /// \brief World position of the object
        /// float4 equals the define xyz
        protected: float4 worldPosition;

        /// \brief World orientation of the object
        protected: Quaternion worldOrientation;

        /// \brief Unique ID of the particle Set
        protected: int particleSetId;

        /// \brief ID of the link set
        protected: int linkSetId;

        /// \brief Object restitution coeficient, 1 - elastic collision , 0 - inelastic
        protected: float restitutionCoef;

        /// \brief sum of all collision forces
        protected: float4 collisionForceSum;

        /// \brief sum of all collision force positions
        protected: float4 collisionForcePosSum;

    };
}
#endif

