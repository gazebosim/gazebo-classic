/* Desc: Fluid World plugin
 * Author: Andrei Haidu
 * Date: 11 May. 2014
 */

#ifndef FLUID_SRV_PLUGIN_HH
#define FLUID_SRV_PLUGIN_HH

#include "FluidEngine.hh"
#include "FluidObjectSet.hh"
#include "FluidSet.hh"

#include "gazebo/gazebo.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/physics.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/Mesh.hh"
#include "gazebo/physics/Base.hh"

#include "boost/bind.hpp"

#include "fluid.pb.h"

namespace gazebo
{
	/// \brief FluidWorldPlugin class
	class FluidWorldPlugin : public WorldPlugin
	{

	/// \brief Constructor
	public: FluidWorldPlugin();

	/// \brief Destructor
	public: ~FluidWorldPlugin();

	/// \brief Load plugin
	protected: void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);

	/// \brief Init plugin (Load called first, then Init)
	protected: void Init();

	/// \brief Update callback function called on every world update event
	protected: void OnUpdate();

	/// \brief Get sdf parameters of the plugin
	protected: void GetSdfParameters(const sdf::ElementPtr &_sdf);

	/// \brief Load fluid engine objects
	protected: void LoadFluids();

	/// \brief Set fluid objects from the loaded models in the world
	protected: void LoadFluidObjects();

	/// \brief check collision geometry type
	protected: void CreateFluidCollision(physics::CollisionPtr &_collision);

	/// \brief pointer to the fluid engine
    private: fluidix::FluidEngine* fluidEngine;

	/// \brief Event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief vector of the fluid particles position
    private: std::vector<float3> fluidParticlesPos;

    /// \brief vector with the all the fluid IDs;
    private: std::vector<int> fluidIDs;

    /// \brief the number of particles in the first fluid
    private: int fluidParticleCount;

    /// \brief world
    private: physics::WorldPtr world;

    /// \brief msg transport node
    private: transport::NodePtr node;

    /// \brief Publisher for fluid visual messages.
    private: transport::PublisherPtr fluidPub;

    /// \brief Publisher for fluid object visual messages.
    private: transport::PublisherPtr fluidObjPub;

    /// \brief map of collisions to the fluidix object ID
    private: std::map<physics::CollisionPtr, int> collisionToObjID_M;

    /// \brief map of collisions to the particle position vector
    private: std::map<physics::CollisionPtr, std::vector<float3> > collisionToParticlesPos_M;

    /// \brief fluid world position
    private: float3 worldPosition;

    /// \brief fluid world size
    private: float3 worldSize;

    /// \brief fluid spawning position
    private: float3 fluidPosition;

    /// \brief fluid volume
    private: float3 spawnVolume;

    /// \brief fluid particles
    private: int particleNr;

	};
}
#endif
