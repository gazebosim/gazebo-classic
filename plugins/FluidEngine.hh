/* Desc: Fluid World plugin
 * Author: Andrei Haidu
 * Email: a.haidu@gmail.com
 * Date: 11 May. 2014
 */

#ifndef FLUID_ENGINE_HH
#define FLUID_ENGINE_HH

#include "vector_types.h"

#include <vector>
#include <map>
#include <string>
#include <list>

#include "FluidObjectSet.hh"
#include "FluidSet.hh"
#include "FluidQuaternion.h"

namespace fluidix
{

    typedef std::map<int, fluidix::FluidSet*> IdToFluidSetMap;

    typedef std::map<int, fluidix::ObjectSet*> IdToObjectSetMap;

    /// \brief FluidEngine class
	class FluidEngine
	{

	/// \brief Constructor
    public: FluidEngine();

    /// \brief Destructor
    public: ~FluidEngine();

    /// \brief Initialize FluidEngineix
    public: void Init();

    /// \brief Update liquid simulation with a new time-step
    public: void Update();

    /// \brief Create a Liquid World with boundary collisions
    public: void CreateWorldBoundaries(float3 _pos, float3 _size, float _elasticity);

    /// \brief Create a new fluid set in the given position, spawned in the given area
    public: void AddFluidSet(float3 _spawn_pos,
    							float3 _size,
    							int _nrNeighbors,
    							float _particleSize,
    							float _massDensity,
    							float _stiffness,
    							float _viscosity,
    							float _buoyancy,
    							float _surfaceTension);

    /// \brief Create a new fluid set in the given position,
    /// spawned in the given area with given particle nr
    public: void AddFluidSet(float3 _spawn_pos,
    							float3 _size,
    							int _particle_nr,
    							int _nrNeighbors,
    							float _particleSize,
    							float _massDensity,
    							float _stiffness,
    							float _viscosity,
    							float _buoyancy,
    							float _surfaceTension);

    /// \brief Return fluid set
    public: FluidSet GetFluidSet(int _id);

    /// \brief Add an Object to the simulation
    public: void AddStaticObject(std::string _path,
							float3 _spawn_pos,
							float3 _scale,
							float _restitution_coeff);

    /// \brief Add a Movable Object to the simulation
    public: int2 AddMovableObject(std::string _path,
									float3 _spawn_pos,
									float4 _spawn_orientation,
									float3 _scale,
									float _restitution_coeff);

    /// \brief Add a Movable Object to the simulation
    public: int2 AddMovableBox(float3 _spawn_pos,
									float4 _spawn_orientation,
									float3 _size,
									float _restitution_coeff);

    /// \brief Return particle positions for the given set
    public: void GetParticlePositions(int _setId,
            std::vector<float3> &_particlePositions);

    /// \brief Return number of Fluid sets
    public: int GetFluidSetCount();

    /// \brief Return number of Object sets
    public: int GetObjectSetCount();

    /// \brief Return number of Movable Object sets
    public: int GetMovableObjectSetCount();

    /// \brief Return particle number for the given set
    public: int GetParticleCount(int _setId);

    /// \brief Retun a vector with all the IDs of the Fluid sets
    public: std::vector<int> GetFluidSetIDs();

    /// \brief Retun a vector with all the IDs of the Object sets
    public: std::vector<int> GetObjectParticleSetIDs();

    /// \brief Retun a vector with all the IDs of the Object sets
    public: std::vector<int> GetMovableObjectParticleSetIDs();

    /// \brief Return the sum of forces acting on the object set
    public: float3 GetObjectCollisionForceSum(int _setId);

    /// \brief Return the position of sum of forces acting on the object set
    public: float3 GetObjectCollisionForcePosSum(int _setId);

    /// \brief Set object's position
    public: void SetObjectPosition(int _setId, float4 _position);

    /// \brief Set object's orientation from quaternion
    public: void SetObjectOrientation(int _setId, Quaternion _quat);

    /// \brief Set object's orientation from euler angles
    public: void SetObjectOrientation(int _setId, float _r, float _p, float _y);

    /// \brief Set object's position
    public: void SetObjectPose(int _setId, float4 _position, Quaternion _quat);


    /// \brief Flag if liquid world is created in order to compute boundary collisions
    private: bool worldBoundariesCreated;

    /// \brief map the fluid set id to itself
    private: IdToFluidSetMap idToFluid_M;

    /// \brief map the static object set id to itself
    private: IdToObjectSetMap idToStaticObject_M;

    /// \brief map the movable object set id to itself
    private: IdToObjectSetMap idToMovableObject_M;

	};
}
#endif
