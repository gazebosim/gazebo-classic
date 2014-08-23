/* Desc: Fluid World plugin
 * Author: Andrei Haidu
 * Date: 11 May. 2014
 */

#include "FluidWorldPlugin.hh"
#include "vector_functions.h"
#include <cmath>

#include <sstream>

#include "FluidQuaternion.h"

using namespace gazebo;

#define PI 3.14159265359


#define UPDATE_THREAD_SLEEP 20 // update thread sleep in milliseconds

//////////////////////////////////////////////////
FluidWorldPlugin::FluidWorldPlugin()
{
	// initialize Fluidix
    this->fluidEngine = new fluidix::FluidEngine();
}

//////////////////////////////////////////////////
FluidWorldPlugin::~FluidWorldPlugin()
{
    delete this->fluidEngine;
}

//////////////////////////////////////////////////
void FluidWorldPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
{
	// get sdf parameters
	FluidWorldPlugin::GetSdfParameters(_sdf);

	// set world pointer
    this->world = _parent;

	// Create Fluid World collision boundaries
	this->fluidEngine->CreateWorldBoundaries(this->worldPosition,			// center position
			this->worldSize,												// size
			0.5f);															// elasticity

    // load fluids
    FluidWorldPlugin::LoadFluids();

	// get the IDs of the fluids
	this->fluidIDs = this->fluidEngine->GetFluidSetIDs();

	// TODO generalize
	// get the number of particles in the first fluid
	if(!this->fluidIDs.empty())
	{
		this->fluidParticleCount = this->fluidEngine->GetParticleCount(this->fluidIDs.front());

		// set the size of the vector to the number of particles in the set
		this->fluidParticlesPos.resize(this->fluidParticleCount);
	}

    // load fluid objects
    FluidWorldPlugin::LoadFluidObjects();

    // resize all the position vectors of the objects
    for (std::map<physics::CollisionPtr, int>::iterator coll_iter = this->collisionToObjID_M.begin();
    		coll_iter != this->collisionToObjID_M.end(); ++coll_iter)
    {
    	// resize the vector with the particle position
    	this->collisionToParticlesPos_M[coll_iter->first] =
    			std::vector<float3>(this->fluidEngine->GetParticleCount(coll_iter->second));
    }

}

//////////////////////////////////////////////////
void  FluidWorldPlugin::Init()
{
	// set up transport node for publishing fluid particles position
    this->node = transport::NodePtr(new transport::Node());
    this->node->Init(this->world->GetName());

    // publisher of the fluid visual
    this->fluidPub = this->node->Advertise<msgs::Fluid>("~/fluid_pos", 10);

    // publisher of the fluid objects visual
    this->fluidObjPub = this->node->Advertise<msgs::Fluid>("~/fluid_obj_pos", 10);

	// after visuals set bind OnUpdate to the ConnectWorldUpdateBegin event
	this->updateConnection = event::Events::ConnectWorldUpdateEnd(
			boost::bind(&FluidWorldPlugin::OnUpdate, this));
}

//////////////////////////////////////////////////
void FluidWorldPlugin::OnUpdate()
{
	// update the fluid engine
	this->fluidEngine->Update();

	// TODO generalize
	// set the fluid particles position
	if (!fluidIDs.empty())
	{
		this->fluidEngine->GetParticlePositions(this->fluidIDs.front(), this->fluidParticlesPos);
	}

	// create fluid message
	msgs::Fluid fluid_msg;
	fluid_msg.set_name("fluid1");

	// get the fluid particle positions
	for (unsigned int i = 0; i < this->fluidParticlesPos.size(); ++i)
	{
		msgs::Set(fluid_msg.add_position(),	math::Vector3(
				this->fluidParticlesPos[i].x,
				this->fluidParticlesPos[i].y,
				this->fluidParticlesPos[i].z));
	}
	// publish the fluid particles positions
 	this->fluidPub->Publish(fluid_msg);


 	// set the new collision positions
	for(std::map<physics::CollisionPtr, std::vector<float3> >::iterator coll_iter = this->collisionToParticlesPos_M.begin();
			coll_iter != this->collisionToParticlesPos_M.end(); ++coll_iter)
	{
		// get model pose
		math::Pose coll_pose = coll_iter->first->GetWorldPose();

		// get the fluidix id of the model
		int obj_id = this->collisionToObjID_M.at(coll_iter->first);

		// apply new pose to fluidix object
		this->fluidEngine->SetObjectPose(obj_id,
				make_float4(coll_pose.pos.x, coll_pose.pos.y, coll_pose.pos.z, 0),
				fluidix::Quaternion(coll_pose.rot.w, coll_pose.rot.x, coll_pose.rot.y, coll_pose.rot.z));


		// visualization part, get all particle positions
		this->fluidEngine->GetParticlePositions(obj_id,
				this->collisionToParticlesPos_M.at(coll_iter->first));

	 	// create obj visualization message
		msgs::Fluid movable_obj_msg;
		movable_obj_msg.set_name(coll_iter->first->GetName().append("_fluid_obj"));

		// TODO check for better perfomance (local var of class attr)?
		for (unsigned int i = 0; i < this->collisionToParticlesPos_M.at(coll_iter->first).size(); ++i)
		{
			msgs::Set(movable_obj_msg.add_position(),
					math::Vector3(this->collisionToParticlesPos_M.at(coll_iter->first)[i].x,
							this->collisionToParticlesPos_M.at(coll_iter->first)[i].y,
							this->collisionToParticlesPos_M.at(coll_iter->first)[i].z));
		}

		// publish the object particles positions
	 	this->fluidObjPub->Publish(movable_obj_msg);
	}

	// apply the forces and torques for every collisions link
	for(std::map<physics::CollisionPtr, int>::iterator map_iter = this->collisionToObjID_M.begin();
			map_iter != this->collisionToObjID_M.end(); ++map_iter)
	{
		// get the forces from the fluid engine
		float3 acc_force_f3 = this->fluidEngine->GetObjectCollisionForceSum(map_iter->second);
		float3 acc_torque_f3 = this->fluidEngine->GetObjectCollisionForcePosSum(map_iter->second);

		// convert forces into gazebo vector3
		math::Vector3 acc_force = math::Vector3(acc_force_f3.x, acc_force_f3.y, acc_force_f3.z);
		math::Vector3 acc_torque = math::Vector3(acc_torque_f3.x, acc_torque_f3.y, acc_torque_f3.z);


		// apply force to the collisions link
		map_iter->first->GetLink()->AddRelativeForce(acc_force);

		// apply torque on the collisions link
		map_iter->first->GetLink()->AddRelativeTorque(acc_torque);
	}
}

//////////////////////////////////////////////////
void FluidWorldPlugin::LoadFluids()
{

	if (this->particleNr == 0)
	{
		/* Spawn water like fluid*/
		this->fluidEngine->AddFluidSet(this->fluidPosition, 			// center position
				this->spawnVolume,										// volume size
				20,														// nr neighbors
				0.02716f,												// particle size / cube side (m) / 10
				998.29f,												// mass density
				3.0f,													// stiffness
				3.5f,													// viscosity
				0.0f,													// buoyancy
				0.0728f);												// surface tension
	}
	else
	{
		this->fluidEngine->AddFluidSet(this->fluidPosition, 			// center position
				this->spawnVolume,										// volume size
				this->particleNr,										// particle nr
				20,														// nr neighbors
				0.02716f,												// particle size / cube side (m) / 10
				998.29f,												// mass density
				3.0f,													// stiffness
				3.5f,													// viscosity
				0.0f,													// buoyancy
				0.0728f);												// surface tension
	}
}

//////////////////////////////////////////////////
void FluidWorldPlugin::LoadFluidObjects()
{
    // get all models from the world
    physics::Model_V _models = this->world->GetModels();

    // iterate through all models
    for (physics::Model_V::iterator m_iter = _models.begin();
    		m_iter != _models.end(); ++m_iter)
    {
    	// get all links from the model
    	physics::Link_V _links = m_iter->get()->GetLinks();

    	std::cout << "Model: " << m_iter->get()->GetName() << std::endl;

    	// iterate through all the links
    	for (physics::Link_V::iterator l_iter = _links.begin();
    			l_iter != _links.end(); ++l_iter)
    	{
    		// get all collisions of the link
    		physics::Collision_V _collisions = l_iter->get()->GetCollisions();

    		std::cout << "\t Link: " << l_iter->get()->GetName() << std::endl;

    		// iterate through all the collisions
    		for (physics::Collision_V::iterator c_iter = _collisions.begin();
    				c_iter != _collisions.end(); ++c_iter)
    		{
        		std::cout << "\t\t Collision: " << (*c_iter)->GetName() << std::endl;

        		// TODO fix all the iter->get stuff
        		physics::CollisionPtr coll_ptr =
        				boost::static_pointer_cast<physics::Collision>(*c_iter);

    			// check the geometry type of the given collision
    			FluidWorldPlugin::CreateFluidCollision(coll_ptr);
    		}
    	}
    }
}

//////////////////////////////////////////////////
void FluidWorldPlugin::CreateFluidCollision(physics::CollisionPtr &_collision)
{
	// sdf element of the collision geometry
	sdf::ElementPtr geom_elem = _collision->GetSDF()->GetElement("geometry");
	// get the name of the geometry
	std::string geom_type = geom_elem->GetFirstElement()->GetName();

	// get the position of the collision
	math::Pose geom_pose = _collision->GetWorldPose();

	// set the position of the geom as float3
	float3 position_f3 = make_float3(geom_pose.pos.x, geom_pose.pos.y, geom_pose.pos.z);

	// set the orientation of the geom as float4
	float4 orientation_f4 = make_float4(
			geom_pose.rot.w, geom_pose.rot.x, geom_pose.rot.y, geom_pose.rot.z);

	// check type of the geometry
	if (geom_type == "box")
	{
		// get the size of the box
		math::Vector3 size =
				geom_elem->GetElement(geom_type)->GetElement("size")->Get<math::Vector3>();

		// set size as float3
		float3 size_f3 = make_float3(size.x, size.y, size.z);

		// insert a movable box shape to the fluid world
		int2 id_set = this->fluidEngine->AddMovableBox(position_f3, orientation_f4, size_f3, 0.05);

		// set model to a constant for the map
		const physics::CollisionPtr const_collision = _collision;

		// map Model to the set ID of the fluid object
		this->collisionToObjID_M[const_collision] = id_set.x;

	}

	else if(geom_type == "cylinder")
	{
		std::cout << "\t\t\tIT'S A CYLINDER" << std::endl;
		gzwarn << "Collision type ["<< geom_type <<"] unimplemented at the moment\n";
	}

	else if(geom_type == "sphere")
	{
		std::cout << "\t\t\tIT'S A SPHERE, radius: "
				<< geom_elem->GetElement(geom_type)->GetElement("radius")->Get<float>() << std::endl;
		gzwarn << "Collision type ["<< geom_type <<"] unimplemented at the moment\n";
	}

	else if(geom_type == "plane")
	{
		std::cout << "\t\t\tIT'S A PLANE, normal: "
				<< geom_elem->GetElement(geom_type)->GetElement("normal")->Get<math::Vector3>()
				<< " ,size: " << geom_elem->GetElement(geom_type)->GetElement("size")->Get<math::Vector2d>()
				<< std::endl;
		gzwarn << "Collision type ["<< geom_type <<"] unimplemented at the moment\n";
	}

	else if(geom_type == "mesh")
	{
		// get the uri element value
		const std::string uri =
				geom_elem->GetElement(geom_type)->GetElement("uri")->Get<std::string>();

		// get the filepath from the uri
		const std::string filepath = common::SystemPaths::Instance()->FindFileURI(uri);

		// get the scale of the mesh
		const math::Vector3 scale_elem =
				geom_elem->GetElement(geom_type)->GetElement("scale")->Get<math::Vector3>();

		// set the scale  of the geom
		float3 scale = make_float3(scale_elem.x, scale_elem.y, scale_elem.z);

		// insert the mesh as movable object into the fluid world
		int2 id_set = this->fluidEngine->AddMovableObject(
				filepath, position_f3, orientation_f4, scale, 0.05);

		// set model to a constant for the map
		const physics::CollisionPtr const_collision = _collision;

		// map Model to the set ID of the fluid object
		this->collisionToObjID_M[const_collision] = id_set.x;
	}

	else
	{
		gzerr << "Collision type ["<< geom_type <<"] unimplemented\n";
	}

}

//////////////////////////////////////////////////
void FluidWorldPlugin::GetSdfParameters(const sdf::ElementPtr &_sdf)
{
	// get parameters from sdf file
	////////////// world_position
	if (!_sdf->HasElement("world_position"))
	{
		std::cout << "Missing parameter <world_position> in FluidWorldPlugin, default to 0 0 0" << std::endl;
		this->worldPosition = make_float3(0, 0, 0);
	}
	else
	{
		math::Vector3 world_position = _sdf->Get<math::Vector3>("world_position");
		this->worldPosition = make_float3(world_position.x, world_position.y, world_position.z);
	}

	////////////// world_size
	if (!_sdf->HasElement("world_size"))
	{
		std::cout << "Missing parameter <world_size> in FluidWorldPlugin, default to 10 10 10" << std::endl;
		this->worldSize = make_float3(10, 10, 10);
	}
	else
	{
		math::Vector3 world_size = _sdf->Get<math::Vector3>("world_size");
		this->worldSize = make_float3(world_size.x, world_size.y, world_size.z);
	}

	////////////// fluid_position
	if (!_sdf->HasElement("fluid_position"))
	{
		std::cout << "Missing parameter <fluid_position> in FluidWorldPlugin, default to 0.0 0.0 1.0" << std::endl;
		this->fluidPosition = make_float3(0.0, 0.0, 1.0);
	}
	else
	{
		math::Vector3 fluid_position = _sdf->Get<math::Vector3>("fluid_position");

		this->fluidPosition = make_float3(fluid_position.x, fluid_position.y, fluid_position.z);
	}

	////////////// fluid_volume
	if (!_sdf->HasElement("fluid_volume"))
	{
		std::cout << "Missing parameter <fluid_volume> in FluidWorldPlugin, default to 0.1 0.1 0.1" << std::endl;
		this->spawnVolume = make_float3(0.1, 0.1, 0.1);
	}
	else
	{
		math::Vector3 fluid_volume = _sdf->Get<math::Vector3>("fluid_volume");

		this->spawnVolume = make_float3(fluid_volume.x, fluid_volume.y, fluid_volume.z);
	}

	////////////// particle_nr
	if (!_sdf->HasElement("particle_nr"))
	{
		std::cout << "Missing parameter <particle_nr> in FluidWorldPlugin, default to 0" << std::endl;
		this->particleNr = 0;
	}
	else
	{
		this->particleNr = _sdf->Get<int>("particle_nr");
	}
}

// Register this plugin with the simulator
GZ_REGISTER_WORLD_PLUGIN(FluidWorldPlugin)






































