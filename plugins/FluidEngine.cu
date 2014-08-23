/* Desc: Fluid World plugin
 * Author: Andrei Haidu
 * Email: a.haidu@gmail.com
 * Date: 11 May. 2014
 */


#include "fluidix.h"
#include "FluidEngine.hh"
#include "math_functions.h"

#include "quaternion.h"

//#define DEBUG
#define GRAVITY 9.80665f 		// gravity (m/s2)
#define DT 0.001 //0.005f 				// time-step (s)
#define MAX_NR_SETS 100 		// maximum nr of sets, global arrays are created with this size

using namespace fluidix;

//////////////////////////////////////////////////
// Global variables
// Needed for external communication with Fluidix

// constants for outside communication with the fluid
struct ParticleSetConstants
{
	float particle_mass;		// particle mass (kg)
	float stiffness;			// pressure stiffness constant (k)
	float density;				// rest density (kg/m3)
	float viscosity;			// viscosity (Pa.s)
	float buoyancy;				// gas buoyancy  (m/s2)
	float threshold;    		// surface tension threshold (4.32)
	float tension;				// surface tension (N/m)
	float smth_neigh_part_nr;	// avg number of neighbors (for smoothing length)
	float h;					// smoothing length (m)
	float restitution; 			// object restitution coef (1 - elastic collision , 0 - inelastic)
};

// object set collision forces and positions, for applying forces on the rigid body physics engine
struct ObjCollisionValues
{
	xyz obj_pos;
	xyz acc_coll_forces;		// sum of forces between objects and particles
	xyz acc_coll_torque;		// sum of force position between objects and particles
};

struct Particle
{
    xyz r, v, v_hstep, f; 	// position, velocity, velocity half step, force (acceleration)
    xyz normal; 			// surface normal
    float density; 			// particle mass-density
    float tension; 			// surface tension force magnitude
};

struct Global
{
	xyz world_pos, world_size; 	// size and position of the world boundaries
	float wall_elasticity; 		// hard wall boundary elasticity
	xyz spawn_pos, spawn_size; 	// spawning particles position, and volume size
	xyz mesh_min, mesh_max; 	// spawning mesh position

	// particle set constants as global array
	ParticleSetConstants *psetdata;

	// object set collision forces and positions
	ObjCollisionValues *collision_values;

	// Default Smoothing Function, its Gradient, and its Laplacian
	inline __host__ __device__ float DefaultKernel(const float dr, const float h){
		return ((315.0f/(64.0f*PI*h*h*h*h*h*h*h*h*h))*((h*h-dr*dr)*(h*h-dr*dr)*(h*h-dr*dr)));
	}
	inline __host__ __device__ float GradDefaultKernel(const float dr, const float h){
		return ((-945.0f/(32.0f*PI*h*h*h*h*h*h*h*h*h))*(h*h-dr*dr)*(h*h-dr*dr)*dr);
	}
	inline __host__ __device__ float LaplacDefaultKernel(const float dr, const float h){
		return (-945.0f/(32.0f*PI*h*h*h*h*h*h*h*h*h))*(h*h-dr*dr)*(3*h*h-7*dr*dr);
	}
	// Pressure Smoothing Function Gradient
	inline __host__ __device__ float GradPressureKernel(const float dr, const float h){
		return (-45.0f/(PI*h*h*h*h*h*h))*(h-dr)*(h-dr);
	}
	// Viscosity Smoothing Function Laplacian
	inline __host__ __device__ float LaplacViscosityKernel(const float dr, const float h){
		return (45.0f/(PI*h*h*h*h*h*h))*(h-dr);
	}
} g;

Fluidix<> *fx;

Particle *particleSet;

//////////////////////////////////////////////////
// Function Macros

// spawn model at given point with given scale
FUNC_EACH(spawn_model,
        p.r = g.mesh_min + p.r * (g.mesh_max - g.mesh_min);
)

// Spawn fluid and init particle density
FUNC_EACH(sph_spawn,
	p.r = make_xyz(rnd_uniform() * g.spawn_size.x, 		// fluid volume X size
				   rnd_uniform() * g.spawn_size.y,		// fluid volume Y size
				   rnd_uniform() * g.spawn_size.z)		// fluid volume Z size
		+ make_xyz(g.spawn_pos.x - g.spawn_size.x/2,	// fluid X position
				   g.spawn_pos.y - g.spawn_size.y/2,	// fluid Y position
				   g.spawn_pos.z - g.spawn_size.z/2);	// fluid Z position
)

// Init SPH values
FUNC_EACH(fluid_sim_init,
	// SPH initialize each particle density
    p.density = g.psetdata[p_set].particle_mass * g.DefaultKernel(0, g.psetdata[p_set].h);

	// re-initialize values
	p.tension = 0;
	p.normal = make_xyz(0, 0, 0);
)

// add density contribution from neighbors
FUNC_PAIR(sph_density,

	float d = g.psetdata[p1_set].particle_mass * g.DefaultKernel(dr, g.psetdata[p1_set].h);

    addFloat(p1.density, d);
    addFloat(p2.density, d);
)

// apply forces on pairs of nearby particles
// pressure and viscosity force
// surface normal and tension
FUNC_PAIR(sph_pair,
	// particle pressure = K(stiffness const) * density diff
    float p1_p = g.psetdata[p1_set].stiffness * (p1.density - g.psetdata[p1_set].density);
    float p2_p = g.psetdata[p2_set].stiffness * (p2.density - g.psetdata[p2_set].density);

    // F pressure:
    xyz vec = u * (g.psetdata[p1_set].particle_mass * g.GradPressureKernel(dr, g.psetdata[p1_set].h)
    		* -(p1_p + p2_p) / (2 * p2.density));

    // F viscosity
    vec += (p2.v - p1.v) * (g.psetdata[p1_set].viscosity * g.LaplacViscosityKernel(dr, g.psetdata[p1_set].h)
    		* g.psetdata[p1_set].particle_mass / p1.density);

    // add forces to the particles
    addVector(p1.f, vec);
    addVector(p2.f, -vec);

    // inward surface normal is the gradient of the color field (4.28)
    vec = u * (g.GradDefaultKernel(dr, g.psetdata[p1_set].h)
    		* g.psetdata[p1_set].particle_mass / p2.density);

    addVector(p1.normal, vec);
    addVector(p2.normal, -vec);

    // F surface tension, a part from (4.31)
    float tens = g.LaplacDefaultKernel(dr, g.psetdata[p1_set].h)
    		* g.psetdata[p1_set].particle_mass / p2.density;

    addFloat(p1.tension, tens);
    addFloat(p2.tension, -tens);
)

// apply forces on each particle
// gravity, buoyancy
FUNC_EACH(sph_each,
	// apply gravity on the particle, this is applied without dependencies on adjacent particles
	// it is acting equally on all fluid particles
    p.f.z -= GRAVITY * p.density;

	// TODO gravity is forgotten
	// p.f.y -= BUOYANCY * (p.density - DENSITY) * GRAVITY;
    p.f.z -= g.psetdata[p_set].buoyancy * (p.density - g.psetdata[p_set].density);

    // F surface tension
    // OBS threshold and magnitude is squared (4.32, 5.17 from kelager06)
    // (should be ||n.i|| >= sqrt(thresh), we have ||n.i||^2 >= thresh, seems fine)
    float normal_magnit_sq = xyz_lensq(p.normal);
    if (normal_magnit_sq > g.psetdata[p_set].threshold) {
    	// F surface tension, the other part of (4.31)
        p.f -= p.normal * (p.tension * g.psetdata[p_set].tension / sqrtf(normal_magnit_sq));
    }
)

// SPH Semi-Implicit Euler integration and re-initialization of values
FUNC_EACH(sph_euler_integrate,
	// integration
	p.v += p.f * DT / p.density;
	p.r += p.v * DT;

	// reset forces to zero being as the timestep ends
	p.f = make_xyz(0, 0, 0);

)

// Leapfrog Integration Step
FUNC_EACH(sph_leapfrog_integrate,
    p.v_hstep += (p.f / p.density) * DT;
	p.v = p.v_hstep + (p.f / p.density) * (DT / 2);
    p.r += p.v_hstep * DT;

	// reset forces to zero being as the timestep ends
    p.f = make_xyz(0, 0, 0);
)

// Object surface collision function
FUNC_SURFACE(surface_collision,

	// compute the velocity on the unit vector direction (p.v * u)
	// compute force before changing the velocity, force = (mass * velocity) / timestep (2nd law)
	float4 p_coll_force = (p.v * g.psetdata[p_set].particle_mass) / DT;
//	float4 p_coll_force = (p.v * u * g.psetdata[p_set].particle_mass) / DT;

//	printf("p_coll_force: %f %f %f,  p_coll_force2: %f %f %f \n",
//			p_coll_force.x, p_coll_force.y, p_coll_force.z,
//			p_coll_force2.x, p_coll_force2.y, p_coll_force2.z);

	// particle position - current position of the object ^(cross_prod) particle coll_force
	float4 p_torque = (p.r -g.collision_values[p1_set].obj_pos) ^ p_coll_force;

	// particle position is projected back along the surface normal (u)
	// with the penetration depth (dr)
	p.r = p.r + dr * u;

	/* Different collision types */
	// Force based
	// p.f += 5000 * u * dr;

	// Standard Hybrid Impulse-Projection method: (4.57)
	// p.v = p.v - (1 + g.psetdata[p_set].restitution) * (p.v % u) * u;

	// Standard Hybrid Impulse-Projection method + (4.58) introducing:
 	// ratio of the penetration depth to the distance between
	// the last particle position and the penetrating position
	/* Semi-Implicit Euler Integration*/
	p.v = p.v - (1 + (g.psetdata[p1_set].restitution * dr)
			/ (DT * xyz_len(p.v))) * (p.v % u) * u;
	/* Leap Frog Integration*/
//	p.v_hstep = p.v_hstep - (1 + (g.psetdata[p_set].restitution * dr)
//			/ (DT * xyz_len(p.v_hstep))) * (p.v_hstep % u) * u;


	// add all particle forces to the global one
	addVector(g.collision_values[p1_set].acc_coll_forces, p_coll_force);

	// add torques to the global one
	addVector(g.collision_values[p1_set].acc_coll_torque, p_torque);

//	printf("set: %d coll_f[%d]: %f %f %f, g.f = %f %f %f  g.pos %f %f %f\n"
//			,p1_set, p_index, coll_force.x, coll_force.y, coll_force.z,
//			g.collision_values[p1_set].sum_coll_forces.x,
//			g.collision_values[p1_set].sum_coll_forces.y,
//			g.collision_values[p1_set].sum_coll_forces.z,
//			g.collision_values[p1_set].sum_coll_forces_pos.x,
//			g.collision_values[p1_set].sum_coll_forces_pos.y,
//			g.collision_values[p1_set].sum_coll_forces_pos.z);

)

// Surface collision with "friction", restitution coefficient
FUNC_SURFACE(static_surface_collision,
	p.r = p.r + dr * u;
	/* Semi-Implicit Euler Integration*/
	p.v = p.v - (1 + (g.psetdata[p_set].restitution * dr) / (DT * xyz_len(p.v))) * (p.v % u) * u;

	p.v *= 0.9;
)

// Surface collision with ray-triangle intersection
FUNC_COLLISION(ray_triangle_collision,
		p.r = p.r + dr * u;
		p.v = p.v - (1 + (g.psetdata[p_set].restitution * dr) / (DT * xyz_len(p.v))) * (p.v % u) * u;
		printf("Force vector: %f %f %f \n",p.f.x, p.f.y, p.f.z);
)

// Collision with Liquid World Boundary
FUNC_EACH(world_boundary,
	// X coord
	// min boundary
    if (p.r.x < g.world_pos.x - (g.world_size.x/2)) {
    	/* Semi-Implicit Euler Integration*/
    	p.v.x = g.wall_elasticity * (g.world_pos.x - (g.world_size.x/2) - p.r.x) / DT;
    	/* Leap Frog Integration*/
    	//p.v_hstep.x = g.wall_elasticity * (g.world_pos.x - (g.world_size.x/2) - p.r.x) / DT;
    	p.r.x = g.world_pos.x - (g.world_size.x/2);
    }
	// max boundary
    if (p.r.x > g.world_pos.x + (g.world_size.x/2)) {
    	/* Semi-Implicit Euler Integration*/
    	p.v.x = g.wall_elasticity * (g.world_pos.x + (g.world_size.x/2)- p.r.x) / DT;
    	/* Leap Frog Integration*/
    	//p.v_hstep.x = g.wall_elasticity * (g.world_pos.x + (g.world_size.x/2)- p.r.x) / DT;
    	p.r.x = g.world_pos.x + (g.world_size.x/2);
    }

    // Y coord
	// min boundary
    if (p.r.y < g.world_pos.y - (g.world_size.y/2)) {
    	/* Semi-Implicit Euler Integration*/
    	p.v.y = g.wall_elasticity * (g.world_pos.y - (g.world_size.y/2) - p.r.y) / DT;
    	/* Leap Frog Integration*/
    	//p.v_hstep.y = g.wall_elasticity * (g.world_pos.y - (g.world_size.y/2) - p.r.y) / DT;
    	p.r.y = g.world_pos.y - (g.world_size.y/2);
    }
	// max boundary
    if (p.r.y > g.world_pos.y + (g.world_size.y/2)) {
    	/* Semi-Implicit Euler Integration*/
    	p.v.y = g.wall_elasticity * (g.world_pos.y + (g.world_size.y/2)- p.r.y) / DT;
    	/* Leap Frog Integration*/
    	//p.v_hstep.y = g.wall_elasticity * (g.world_pos.y + (g.world_size.y/2)- p.r.y) / DT;
    	p.r.y = g.world_pos.y + (g.world_size.y/2);
    }

    // Z coord
	// min boundary
    if (p.r.z < g.world_pos.z - (g.world_size.z/2)) {
    	/* Semi-Implicit Euler Integration*/
    	p.v.z = g.wall_elasticity * (g.world_pos.z - (g.world_size.z/2) - p.r.z) / DT;
    	/* Leap Frog Integration*/
    	//p.v_hstep.z = g.wall_elasticity * (g.world_pos.z - (g.world_size.z/2) - p.r.z) / DT;
    	p.r.z = g.world_pos.z - (g.world_size.z/2);
    }
	// max boundary
    if (p.r.z > g.world_pos.z + (g.world_size.z/2)) {
    	/* Semi-Implicit Euler Integration*/
    	p.v.z = g.wall_elasticity * (g.world_pos.z + (g.world_size.z/2)- p.r.z) / DT;
    	/* Leap Frog Integration*/
    	//p.v_hstep.z = g.wall_elasticity * (g.world_pos.z + (g.world_size.z/2)- p.r.z) / DT;
    	p.r.z = g.world_pos.z + (g.world_size.z/2);
    }
)

//////////////////////////////////////////////////
FluidEngine::FluidEngine()
{
	// Initialize Fluidix
    fx = new Fluidix<>(&g);

    // Initialize members
    worldBoundariesCreated = false;

    // Init global arrays
    fx->createGlobalArray(&g.psetdata, MAX_NR_SETS * sizeof(ParticleSetConstants));

	// Init global array of object set collision forces and positions
    fx->createGlobalArray(&g.collision_values, MAX_NR_SETS * sizeof(ObjCollisionValues));
}

//////////////////////////////////////////////////
FluidEngine::~FluidEngine()
{
// TODO if destructor is called before last CUDA update it crashes badly
//	delete fx;
}

//////////////////////////////////////////////////
void FluidEngine::Init()
{

}

//////////////////////////////////////////////////
void FluidEngine::Update()
{
    fx->setTimer();

    for (IdToFluidSetMap::iterator fluid_iter = this->idToFluid_M.begin();
            fluid_iter != this->idToFluid_M.end(); fluid_iter++)
    {
    	// Init SPH values
    	fx->runEach(fluid_sim_init(),
    			fluid_iter->second->GetParticleSetId()); 				// particle Set of the Fluid

    	// add density contribution from neighbors
    	fx->runPair(sph_density(),
    			fluid_iter->second->GetParticleSetId(), 				// particle Set of the Fluid
    			fluid_iter->second->GetParticleSetId(), 				// particle Set of the Fluid
    			g.psetdata[fluid_iter->second->GetParticleSetId()].h); 	// smoothing length of of the Fluid

    	// apply forces on pairs of nearby particles, pressure, viscosity and surface tension force
    	fx->runPair(sph_pair(),
    			fluid_iter->second->GetParticleSetId(), 				// particle Set of the Fluid
    			fluid_iter->second->GetParticleSetId(), 				// particle Set of the Fluid
    			g.psetdata[fluid_iter->second->GetParticleSetId()].h);	// smoothing length of the Fluid

    	// apply forces on each particle, gravity and buoyancy
    	fx->runEach(sph_each(),
    			fluid_iter->second->GetParticleSetId()); 		// particle Set of the Fluid



    	// if world boundaries are created interact with them
    	if (this->worldBoundariesCreated)
    	{
    		fx->runEach(world_boundary(),
    				fluid_iter->second->GetParticleSetId()); 	// particle Set of the Fluid
    	}

        // interact with all STATIC Objects
        for (IdToObjectSetMap::iterator object_iter = this->idToStaticObject_M.begin();
                object_iter != this->idToStaticObject_M.end(); object_iter++)
        {
            // apply surface collision with static object
            fx->runSurface(static_surface_collision(),
                    object_iter->second->GetLinkId(), 			// link Set of the Object
                    fluid_iter->second->GetParticleSetId(),		// particle Set of the Object
                    -1);										// -1 = interact with all depth sizes

        }

        // interact with all MOVABLE Objects
        for (IdToObjectSetMap::iterator movable_object_iter = this->idToMovableObject_M.begin();
                movable_object_iter != this->idToMovableObject_M.end(); movable_object_iter++)
        {
//        	fx->runCollision(ray_triangle_collision(),
//                    movable_object_iter->second->GetLinkId(), 	// link Set of the Movable Object
//                    fluid_iter->second->GetParticleSetId(),		// particle Set of the Movable Object
//                    DT);										// time step for the ray start point r-v*dt

            // apply surface collision with movable object
            fx->runSurface(surface_collision(),
                    movable_object_iter->second->GetLinkId(), 	// link Set of the Movable Object
                    fluid_iter->second->GetParticleSetId(),		// particle Set of the Movable Object
                    -1);										// -1 = interact with all depth sizes
        }

    	// Semi Impliciy Euler integration
    	fx->runEach(sph_euler_integrate(),
    			fluid_iter->second->GetParticleSetId()); 		// particle Set of the Fluid

    	// Leapfrog integration
    	// fx->runEach(sph_leapfrog_integrate(),
    	// fluid_iter->second->GetParticleSetId()); 			// particle Set of the Fluid
    }


    // get global collision data (forces/forces position) from the gpu memory
    fx->getGlobalArray(&g.collision_values);

    // loop through all the objects to add the values to the objects and reset them
    for (IdToObjectSetMap::iterator movable_object_iter = this->idToMovableObject_M.begin();
            movable_object_iter != this->idToMovableObject_M.end(); movable_object_iter++)
    {
    	// set the sum of forces and positions from the global gpu memory
    	movable_object_iter->second->SetCollisionForceSum(
    			g.collision_values[movable_object_iter->first].acc_coll_forces);
    	movable_object_iter->second->SetCollisionForcePosSum(
    			g.collision_values[movable_object_iter->first].acc_coll_torque);

    	// reset the global values of the sums
    	g.collision_values[movable_object_iter->first].acc_coll_forces = make_xyz(0,0,0);
    	g.collision_values[movable_object_iter->first].acc_coll_torque = make_xyz(0,0,0);
    	g.collision_values[movable_object_iter->first].obj_pos =
    			movable_object_iter->second->GetWorldPosition();
    }

    // set the reseted collision data to the gpu memory
    fx->applyGlobalArray(&g.collision_values);



    // get the durration of the computation in order to compare it to the real time factor
    float comp_durr = fx->getTimer();

    printf("*GPU* Update dur: %.2f ms;\n", comp_durr);
}

//////////////////////////////////////////////////
void FluidEngine::CreateWorldBoundaries(float3 _pos, float3 _size, float _elasticity)
{
	// set flag in order to compute collisions with the world boundaries
	this->worldBoundariesCreated = true;

	// Set world position
	g.world_pos = make_xyz(_pos.x, _pos.y, _pos.z);

	// Set world boundary size
	g.world_size = make_xyz(_size.x, _size.y, _size.z);

	// Set hard wall boundary elasticity
	g.wall_elasticity = _elasticity;
}

//////////////////////////////////////////////////
void FluidEngine::AddFluidSet(float3 _spawn_pos,
								  float3 _volume_size,
								  int _nrNeighbors,
								  float _particleSize,
								  float _massDensity,
								  float _stiffness,
								  float _viscosity,
								  float _buoyancy,
								  float _surfaceTension)
{
	int particle_nr;
	float h;

	// local instance of the fluid class
	fluidix::FluidSet *fluidSet =  new fluidix::FluidSet();

	// set fluid volume (m3)
	fluidSet->SetVolume(_volume_size.x * _volume_size.y * _volume_size.z);

	// set particle size (m) as side of a cube
	fluidSet->SetParticleSize(_particleSize);

	// compute and set particle numbers
	particle_nr = fluidSet->GetVolume() / fluidSet->GetParticleVolume();
	fluidSet->SetParticleNr(particle_nr);

	// set avg number of neighbors (for smoothing length)
	fluidSet->SetSmoothingNeighborsNr(_nrNeighbors);

	// TODO if GetVolume is the particle volume, or the whole fluid
	// set smoothing length
	h = powf(3.0f* fluidSet->GetVolume() * fluidSet->GetSmoothingNeighboursNr() /
			(4.0f * PI * fluidSet->GetParticleNr()), 0.333f);
//	h = powf((0.75f/PI) * 20 * 0.02f / 998.29f, 0.333f);

	fluidSet->SetSmoothingLength(h);

	// set fluid density
	fluidSet->SetDensity(_massDensity);

	// set fluid stiffness
	fluidSet->SetStiffness(_stiffness);

	// set fluid viscosity
	fluidSet->SetViscosity(_viscosity);

	// set fluid buoyancu
	fluidSet->SetBuoyancy(_buoyancy);

	// set fluid surface tension
	fluidSet->SetSurfaceTension(_surfaceTension);

	// set fluid surface tension threshold
	fluidSet->SetSurfTensThreshold(_massDensity / _nrNeighbors);

	// set particle mass
	fluidSet->SetParticleMass(fluidSet->GetDensity() * fluidSet->GetVolume() /
			fluidSet->GetParticleNr());


	// get unique Id and create particle set
	int unique_id = fx->createParticleSet(particle_nr);

	// set ParticleSet unique ID
	fluidSet->SetParticleSetId(unique_id);

	// add particle set to the map
	this->idToFluid_M[unique_id] = fluidSet;


	// Add the Fluid parameters to the global array
	g.psetdata[unique_id].particle_mass = fluidSet->GetParticleMass();

	g.psetdata[unique_id].stiffness = fluidSet->GetStiffness();

	g.psetdata[unique_id].density = fluidSet->GetDensity();

	g.psetdata[unique_id].viscosity = fluidSet->GetViscosity();

	g.psetdata[unique_id].buoyancy = fluidSet->GetBuoyancy();

	g.psetdata[unique_id].tension = fluidSet->GetSurfaceTension();

	g.psetdata[unique_id].threshold = fluidSet->GetSurfTensThreshold();

	g.psetdata[unique_id].h = fluidSet->GetSmoothingLength();

	fx->applyGlobalArray(&g.psetdata);

	// set spawning position
    g.spawn_pos = make_xyz(_spawn_pos.x, _spawn_pos.y, _spawn_pos.z);

    // set spawning area
    g.spawn_size  = make_xyz(_volume_size.x, _volume_size.y, _volume_size.z);

    // Spawn fluid and init particle density for SPH
    fx->runEach(sph_spawn(), unique_id);

    // Print Info about the Fluid
    std::cout << "** Fluid set nr: " << unique_id <<", parameters:" << std::endl;
    std::cout << "Particle Nr: " << particle_nr << std::endl;
    std::cout << "Neighbours: " << _nrNeighbors << std::endl;
    std::cout << "Particle Mass: " << fluidSet->GetParticleMass() << std::endl;
    std::cout << "Stiffness: " << fluidSet->GetStiffness() << std::endl;
    std::cout << "Mass Density: " << fluidSet->GetDensity() << std::endl;
    std::cout << "Viscosity: " << fluidSet->GetViscosity() << std::endl;
    std::cout << "Buoyancy: " << fluidSet->GetBuoyancy() << std::endl;
    std::cout << "Surface Tension: " << fluidSet->GetSurfaceTension() << std::endl;
    std::cout << "Surface Tension Threshold (squared): " << fluidSet->GetSurfTensThreshold() << std::endl;
    std::cout << "Smoothing length (h): " << fluidSet->GetSmoothingLength() << std::endl;

}

//////////////////////////////////////////////////
void FluidEngine::AddFluidSet(float3 _spawn_pos,
								  float3 _volume_size,
								  int _particle_nr,
								  int _nrNeighbors,
								  float _particleSize,
								  float _massDensity,
								  float _stiffness,
								  float _viscosity,
								  float _buoyancy,
								  float _surfaceTension)
{
	float h;

	// local instance of the fluid class
	fluidix::FluidSet *fluidSet =  new fluidix::FluidSet();

	// set fluid volume (m3)
	fluidSet->SetVolume(_volume_size.x * _volume_size.y * _volume_size.z);

	// set particle size (m) as side of a cube
	fluidSet->SetParticleSize(_particleSize);

	// set particle numbers
	fluidSet->SetParticleNr(_particle_nr);

	// set avg number of neighbors (for smoothing length)
	fluidSet->SetSmoothingNeighborsNr(_nrNeighbors);

	// TODO if GetVolume is the particle volume, or the whole fluid
	// set smoothing length
	h = powf(3.0f* (fluidSet->GetParticleVolume()*_particle_nr) * fluidSet->GetSmoothingNeighboursNr() /
			(4.0f * PI * fluidSet->GetParticleNr()), 0.333f);
//	h = powf((0.75f/PI) * 20 * 0.02f / 998.29f, 0.333f);

	fluidSet->SetSmoothingLength(h);

	// set fluid density
	fluidSet->SetDensity(_massDensity);

	// set fluid stiffness
	fluidSet->SetStiffness(_stiffness);

	// set fluid viscosity
	fluidSet->SetViscosity(_viscosity);

	// set fluid buoyancy
	fluidSet->SetBuoyancy(_buoyancy);

	// set fluid surface tension
	fluidSet->SetSurfaceTension(_surfaceTension);

	// set fluid surface tension threshold
	fluidSet->SetSurfTensThreshold(_massDensity / _nrNeighbors);

	// TODO compute particle mass for custom particle nr as well
	// set particle mass
//	fluidSet->SetParticleMass(fluidSet->GetDensity() * fluidSet->GetVolume() /
//			fluidSet->GetParticleNr());
	fluidSet->SetParticleMass(fluidSet->GetDensity() * fluidSet->GetParticleVolume()*_particle_nr /
			fluidSet->GetParticleNr());

	// get unique Id and create particle set
	int unique_id = fx->createParticleSet(_particle_nr);

	// set ParticleSet unique ID
	fluidSet->SetParticleSetId(unique_id);

	// add particle set to the map
	this->idToFluid_M[unique_id] = fluidSet;


	// Add the Fluid parameters to the global array
	g.psetdata[unique_id].particle_mass = fluidSet->GetParticleMass();

	g.psetdata[unique_id].stiffness = fluidSet->GetStiffness();

	g.psetdata[unique_id].density = fluidSet->GetDensity();

	g.psetdata[unique_id].viscosity = fluidSet->GetViscosity();

	g.psetdata[unique_id].buoyancy = fluidSet->GetBuoyancy();

	g.psetdata[unique_id].tension = fluidSet->GetSurfaceTension();

	g.psetdata[unique_id].threshold = fluidSet->GetSurfTensThreshold();

	g.psetdata[unique_id].h = fluidSet->GetSmoothingLength();

	fx->applyGlobalArray(&g.psetdata);

	// set spawning position
    g.spawn_pos = make_xyz(_spawn_pos.x, _spawn_pos.y, _spawn_pos.z);

    // set spawning area
    g.spawn_size  = make_xyz(_volume_size.x, _volume_size.y, _volume_size.z);

    // Spawn fluid and init particle density for SPH
    fx->runEach(sph_spawn(), unique_id);

    // Print Info about the Fluid
    std::cout << "** Fluid set nr: " << unique_id <<", parameters:" << std::endl;
    std::cout << "Particle Nr: " << _particle_nr << std::endl;
    std::cout << "Neighbours: " << _nrNeighbors << std::endl;
    std::cout << "Particle Mass: " << fluidSet->GetParticleMass() << std::endl;
    std::cout << "Stiffness: " << fluidSet->GetStiffness() << std::endl;
    std::cout << "Mass Density: " << fluidSet->GetDensity() << std::endl;
    std::cout << "Viscosity: " << fluidSet->GetViscosity() << std::endl;
    std::cout << "Buoyancy: " << fluidSet->GetBuoyancy() << std::endl;
    std::cout << "Surface Tension: " << fluidSet->GetSurfaceTension() << std::endl;
    std::cout << "Surface Tension Threshold (squared): " << fluidSet->GetSurfTensThreshold() << std::endl;
    std::cout << "Smoothing length (h): " << fluidSet->GetSmoothingLength() << std::endl;

}

//////////////////////////////////////////////////
FluidSet FluidEngine::GetFluidSet(int _id)
{
	return *this->idToFluid_M.find(_id)->second;
}

//////////////////////////////////////////////////
void FluidEngine::AddStaticObject(std::string _path,
							float3 _spawn_pos,
							float3 _scale,
							float _restitution_coeff)
{
    // local instance of the object class
    fluidix::ObjectSet *objectSet = new fluidix::ObjectSet();

    // import model and set particle/link set unique IDs
    int2 object_ids = fx->importModel(_path.c_str());

    // set Objects Particle and Link Set IDs
    // .x = particle set index , .y link set index
    objectSet->SetParticleAndLinkSetId(object_ids.x, object_ids.y);

    // save the initial position of the object
    objectSet->SetWorldPosition(_spawn_pos);

    // TODO hardcoded
    // save the initial orientation of the object
    objectSet->SetWorldOrientation(Quaternion(1,0,0,0));

    // set restitution coefficient, // 1 - elastic collision , 0 - inelastic
    objectSet->SetRestitutionCoef(_restitution_coeff); //TODO hardcoded

    // add Object to the map, unique Id is the Particle Set ID
    this->idToStaticObject_M[object_ids.x] = objectSet;

    // set the restitution coefficient for this object
	g.psetdata[object_ids.x].restitution = objectSet->GetRestitutionCoef();
	fx->applyGlobalArray(&g.psetdata);


    // Set mesh spawning position
    g.mesh_min = make_xyz(_spawn_pos.x, _spawn_pos.y, _spawn_pos.z);
    g.mesh_max = make_xyz(_spawn_pos.x + _scale.x, _spawn_pos.y + _scale.y, _spawn_pos.z + _scale.z);

    // .x = particle set index , .y link set index
    fx->runEach(spawn_model(), object_ids.x);
}

//////////////////////////////////////////////////
int2 FluidEngine::AddMovableObject(std::string _path,
		float3 _spawn_pos,
		float4 _orientation,
		float3 _scale,
		float _restitution_coeff)
{
    // local instance of the object class
    fluidix::ObjectSet *objectSet = new fluidix::ObjectSet();

    // import model and set particle/link set unique IDs
    int2 id_set = fx->importModel(_path.c_str());

    // set Objects Particle and Link Set IDs
    // .x = particle set index , .y link set index
    objectSet->SetParticleAndLinkSetId(id_set.x, id_set.y);

    // save the initial position of the object
    objectSet->SetWorldPosition(_spawn_pos);

    // TODO check why initial orientation has to be ht unit one, otherwise it's mirrored
    // save the initial orientation of the object
    objectSet->SetWorldOrientation(Quaternion(1,0,0,0));
//    objectSet->SetWorldOrientation(Quaternion(_orientation));

    // set restitution coefficient, // 1 - elastic collision , 0 - inelastic
    objectSet->SetRestitutionCoef(_restitution_coeff);

    // add Object to the map, unique Id is the Particle Set ID
    this->idToMovableObject_M[id_set.x] = objectSet;

    // set the restitution coefficient for this object
	g.psetdata[id_set.x].restitution = objectSet->GetRestitutionCoef();
	fx->applyGlobalArray(&g.psetdata);


    // Set mesh spawning position
    g.mesh_min = make_xyz(_spawn_pos.x, _spawn_pos.y, _spawn_pos.z);
    g.mesh_max = make_xyz(_spawn_pos.x + _scale.x, _spawn_pos.y + _scale.y, _spawn_pos.z + _scale.z);

    // .x = particle set index , .y link set index
    fx->runEach(spawn_model(), id_set.x);

    return id_set;
}

//////////////////////////////////////////////////
int2 FluidEngine::AddMovableBox(float3 _spawn_pos,
		float4 _orientation,
		float3 _size,
		float _restitution_coeff)
{
    // local instance of the object class
    fluidix::ObjectSet *objectSet = new fluidix::ObjectSet();

    // unique ID of the particle/link Set
    // .x = particle set index , .y link set index
    int2 box_ids;

    // manually define the vertices of the box shape
    box_ids.x = fx->createParticleSet(8);
    // get the array of particles from the box
    Particle *p = fx->getParticleArray(box_ids.x);

    // create a unit box
	p[0].r = make_xyz(-0.5, -0.5, -0.5);
	p[1].r = make_xyz(-0.5, -0.5,  0.5);
	p[2].r = make_xyz(-0.5,  0.5, -0.5);
	p[3].r = make_xyz(-0.5,  0.5,  0.5);
	p[4].r = make_xyz( 0.5, -0.5, -0.5);
	p[5].r = make_xyz( 0.5, -0.5,  0.5);
	p[6].r = make_xyz( 0.5,  0.5, -0.5);
	p[7].r = make_xyz( 0.5,  0.5,  0.5);

	// apply changes to the array
	fx->applyParticleArray(box_ids.x);

	// define links to form triangles
	box_ids.y = fx->createLinkSet();

	fx->addLink(box_ids.y, box_ids.x, 0, box_ids.x, 1);
	fx->addLink(box_ids.y, box_ids.x, 2, box_ids.x, 3);
	fx->addLink(box_ids.y, box_ids.x, 0, box_ids.x, 2);
	fx->addLink(box_ids.y, box_ids.x, 1, box_ids.x, 3);
	fx->addLink(box_ids.y, box_ids.x, 4, box_ids.x, 5);
	fx->addLink(box_ids.y, box_ids.x, 6, box_ids.x, 7);
	fx->addLink(box_ids.y, box_ids.x, 4, box_ids.x, 6);
	fx->addLink(box_ids.y, box_ids.x, 5, box_ids.x, 7);
	fx->addLink(box_ids.y, box_ids.x, 0, box_ids.x, 4);
	fx->addLink(box_ids.y, box_ids.x, 1, box_ids.x, 5);
	fx->addLink(box_ids.y, box_ids.x, 2, box_ids.x, 6);
	fx->addLink(box_ids.y, box_ids.x, 3, box_ids.x, 7);
	fx->addLink(box_ids.y, box_ids.x, 0, box_ids.x, 3);
	fx->addLink(box_ids.y, box_ids.x, 4, box_ids.x, 7);
	fx->addLink(box_ids.y, box_ids.x, 0, box_ids.x, 5);
	fx->addLink(box_ids.y, box_ids.x, 2, box_ids.x, 7);
	fx->addLink(box_ids.y, box_ids.x, 0, box_ids.x, 6);
	fx->addLink(box_ids.y, box_ids.x, 1, box_ids.x, 7);


    // set Objects Particle and Link Set IDs
    // .x = particle set index , .y link set index
    objectSet->SetParticleAndLinkSetId(box_ids);

    // save the initial position of the object
    objectSet->SetWorldPosition(_spawn_pos);

    // TODO check why initial orientation has to be ht unit one, otherwise it's mirrored
    // save the initial orientation of the object
    objectSet->SetWorldOrientation(Quaternion(1,0,0,0));
//    objectSet->SetWorldOrientation(Quaternion(_orientation));

    // set restitution coefficient, // 1 - elastic collision , 0 - inelastic
    objectSet->SetRestitutionCoef(_restitution_coeff);

    // add Object to the map, unique Id is the Particle Set ID
    this->idToMovableObject_M[box_ids.x] = objectSet;

    // set the restitution coefficient for this object
	g.psetdata[box_ids.x].restitution = objectSet->GetRestitutionCoef();
	fx->applyGlobalArray(&g.psetdata);


    // Set box spawning position and size
    g.mesh_min = make_xyz(_spawn_pos.x, _spawn_pos.y, _spawn_pos.z);
    g.mesh_max = make_xyz(_spawn_pos.x + _size.x, _spawn_pos.y + _size.y, _spawn_pos.z + _size.z);

    // .x = particle set index , .y link set index
    fx->runEach(spawn_model(), box_ids.x);

    return box_ids;
}

//////////////////////////////////////////////////
void FluidEngine::GetParticlePositions(int _setId,
    		std::vector<float3> &_particlePositions)
{
    // get the particle set
    particleSet = fx->getParticleArray(_setId);

    // get the nr of particles
    int p_count = fx->getParticleCount(_setId);

    // write every particle postition to the vector
    for (int p_i = 0; p_i < p_count; ++p_i)
    {
    	_particlePositions[p_i].x = particleSet[p_i].r.x;
    	_particlePositions[p_i].y = particleSet[p_i].r.y;
    	_particlePositions[p_i].z = particleSet[p_i].r.z;
    }
}

//////////////////////////////////////////////////
int FluidEngine::GetFluidSetCount()
{
	return this->idToFluid_M.size();
}

//////////////////////////////////////////////////
int FluidEngine::GetObjectSetCount()
{
    return this->idToStaticObject_M.size();
}

//////////////////////////////////////////////////
int FluidEngine::GetMovableObjectSetCount()
{
    return this->idToMovableObject_M.size();
}

//////////////////////////////////////////////////
int FluidEngine::GetParticleCount(int _setId)
{
	return fx->getParticleCount(_setId);
}

//////////////////////////////////////////////////
std::vector<int> FluidEngine::GetFluidSetIDs()
{
	std::vector<int> IDs;

    for (IdToFluidSetMap::iterator iter = this->idToFluid_M.begin();
    		iter != this->idToFluid_M.end(); iter++)
    {
    	IDs.push_back(iter->second->GetParticleSetId());
    }

	return IDs;
}

//////////////////////////////////////////////////
std::vector<int> FluidEngine::GetObjectParticleSetIDs()
{
    std::vector<int> IDs;

    for (IdToObjectSetMap::iterator iter = this->idToStaticObject_M.begin();
            iter != this->idToStaticObject_M.end(); iter++)
    {
        IDs.push_back(iter->second->GetParticleSetId());
    }

    return IDs;
}

//////////////////////////////////////////////////
std::vector<int> FluidEngine::GetMovableObjectParticleSetIDs()
{
    std::vector<int> IDs;

    for (IdToObjectSetMap::iterator iter = this->idToMovableObject_M.begin();
            iter != this->idToMovableObject_M.end(); iter++)
    {
        IDs.push_back(iter->second->GetParticleSetId());
    }

    return IDs;
}

//////////////////////////////////////////////////
float3 FluidEngine::GetObjectCollisionForceSum(int _setId)
{
	xyz sum_force = this->idToMovableObject_M[_setId]->GetCollisionForceSum();

	return make_float3(sum_force.x, sum_force.y, sum_force.z);
}

//////////////////////////////////////////////////
float3 FluidEngine::GetObjectCollisionForcePosSum(int _setId)
{
	xyz sum_force_pos = this->idToMovableObject_M[_setId]->GetCollisionForcePosSum();

	return make_float3(sum_force_pos.x, sum_force_pos.y, sum_force_pos.z);
}

//////////////////////////////////////////////////
void FluidEngine::SetObjectPosition(int _setId, float4 _position)
{
    // get the particle set
    particleSet = fx->getParticleArray(_setId);

    // calculate the relative position
    float4 diff = _position - this->idToMovableObject_M[_setId]->GetWorldPosition();

    // get number of particles
    int p_count = fx->getParticleCount(_setId);

    // iterate through the particle set
    for (int i = 0; i < p_count; ++i)
    {
        particleSet[i].r += diff;
    }

    // update object new position
    this->idToMovableObject_M[_setId]->SetWorldPosition(_position);

    // apply changes made to particles outside of an interaction
    fx->applyParticleArray(_setId);

}

//////////////////////////////////////////////////
void FluidEngine::SetObjectOrientation(int _setId, Quaternion _quat)
{
	// get the world position, to translate it to the center and then back
	xyz world_pos = this->idToMovableObject_M[_setId]->GetWorldPosition();

	// the new quat * the inverse of the last position so it takes it back to the init position
	Quaternion quat = _quat * this->idToMovableObject_M[_setId]->GetWorldOrientation().GetInverse();

    // get the particle set
    particleSet = fx->getParticleArray(_setId);

    // get number of particles
    int p_count = fx->getParticleCount(_setId);

    // iterate through the particle set
    for (int i = 0; i < p_count; ++i)
    {
    	// rotate the translated position vector
    	particleSet[i].r = quat.RotateVector(particleSet[i].r - world_pos);

    	// translate back the position vector by adding back its original position
    	particleSet[i].r += world_pos;

    }

    // update object new position
    this->idToMovableObject_M[_setId]->SetWorldOrientation(_quat);

    // apply changes made to particles outside of an interaction
    fx->applyParticleArray(_setId);

}

//////////////////////////////////////////////////
void FluidEngine::SetObjectOrientation(int _setId, float _r, float _p, float _y)
{
	Quaternion quat(_r, _p, _y);

	this->SetObjectOrientation(_setId, quat);
}

// TODO overload method with different parameter types (eg, float4 instead of quat)
//////////////////////////////////////////////////
void FluidEngine::SetObjectPose(int _setId, float4 _position, Quaternion _quat)
{
	// get the world position, to translate it to the center and then back
	xyz world_pos = this->idToMovableObject_M[_setId]->GetWorldPosition();

	// the new quat * the inverse of the last position so it takes it back to the init position
	Quaternion quat = _quat * this->idToMovableObject_M[_setId]->GetWorldOrientation().GetInverse();

	// get the particle set
    particleSet = fx->getParticleArray(_setId);

    // get number of particles
    int p_count = fx->getParticleCount(_setId);

    // iterate through the particle set
    for (int i = 0; i < p_count; ++i)
    {
    	// rotate the translated position vector
    	particleSet[i].r = quat.RotateVector(particleSet[i].r - world_pos);

    	// translate back the position to the sensor value
    	particleSet[i].r += _position;
    }

    // update object new position
    this->idToMovableObject_M[_setId]->SetWorldPosition(_position);

    // update object new position
    this->idToMovableObject_M[_setId]->SetWorldOrientation(_quat);

    // apply changes made to particles outside of an interaction
    fx->applyParticleArray(_setId);
}




