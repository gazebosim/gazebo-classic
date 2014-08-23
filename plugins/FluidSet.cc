/* Desc: Fluid Set class
 * Author: Andrei Haidu
 * Date: 11 Jul. 2013
 */

#include "FluidSet.hh"

using namespace fluidix;

//////////////////////////////////////////////////
FluidSet::FluidSet()
{
	// set sentinel value to check afterwards if the members have been initialized
	this->particleMass = -1;
	this->density = -1;
	this->volume = -1;
	this->particleNr = -1;

}

//////////////////////////////////////////////////
FluidSet::~FluidSet()
{

}

//////////////////////////////////////////////////
void FluidSet::Load()
{

}

//////////////////////////////////////////////////
void FluidSet::Fini()
{

}

//////////////////////////////////////////////////
void FluidSet::Init()
{

}

//////////////////////////////////////////////////
void FluidSet::Reset()
{
	// Re-set the sentinel values
	this->particleMass = -1;
	this->density = -1;
	this->volume = -1;
	this->particleNr = -1;

}

//////////////////////////////////////////////////
void FluidSet::Update()
{

}

//////////////////////////////////////////////////
void FluidSet::SetParticleSetId(int _id)
{
	this->particleSetId = _id;
}

//////////////////////////////////////////////////
int FluidSet::GetParticleSetId()
{
	return this->particleSetId;
}

//////////////////////////////////////////////////
void FluidSet::SetParticleNr(int _nr)
{
	this->particleNr = _nr;
}

//////////////////////////////////////////////////
int FluidSet::GetParticleNr()
{
	return this->particleNr;
}

//////////////////////////////////////////////////
void FluidSet::SetVolume(float _volume)
{
	this->volume = _volume;
}

//////////////////////////////////////////////////
float FluidSet::GetVolume()
{
	return this->volume;
}

//////////////////////////////////////////////////
void FluidSet::SetParticleSize(float _size)
{
	this->particleSize = _size;
}

//////////////////////////////////////////////////
float FluidSet::GetParticleSize()
{
	return this->particleSize;
}

//////////////////////////////////////////////////
float FluidSet::GetParticleVolume()
{
	return this->particleSize * this->particleSize * this->particleSize;
}


//////////////////////////////////////////////////
void FluidSet::SetDensity(float _density)
{
	this->density = _density;
}

//////////////////////////////////////////////////
float FluidSet::GetDensity()
{
	return this->density;
}

//////////////////////////////////////////////////
void FluidSet::SetParticleMass(float _mass)
{
	this->particleMass = _mass;
}

//////////////////////////////////////////////////
float FluidSet::GetParticleMass()
{
	return this->particleMass;
}

//////////////////////////////////////////////////
void FluidSet::SetFluidMass(float _mass)
{
	this->fluidMass = _mass;
}

//////////////////////////////////////////////////
float FluidSet::GetFluidMass()
{
	return this->fluidMass;
}

//////////////////////////////////////////////////
void FluidSet::SetStiffness(float _stiffness)
{
	this->stiffness = _stiffness;
}

//////////////////////////////////////////////////
float FluidSet::GetStiffness()
{
	return this->stiffness;
}

//////////////////////////////////////////////////
void FluidSet::SetViscosity(float _viscosity)
{
	this->viscosity = _viscosity;
}

//////////////////////////////////////////////////
float FluidSet::GetViscosity()
{
	return this->viscosity;
}

//////////////////////////////////////////////////
void FluidSet::SetSurfaceTension(float _tension)
{
	this->surfaceTension = _tension;
}

//////////////////////////////////////////////////
float FluidSet::GetSurfaceTension()
{
	return this->surfaceTension;
}

//////////////////////////////////////////////////
void FluidSet::SetSurfTensThreshold(float _surfTensThreshold)
{
	this->surfTensThreshold = _surfTensThreshold;
}

//////////////////////////////////////////////////
float FluidSet::GetSurfTensThreshold()
{
	return this->surfTensThreshold;
}

//////////////////////////////////////////////////
void FluidSet::SetBuoyancy(float _buoyancy)
{
	this->buoyancy = _buoyancy;
}

//////////////////////////////////////////////////
float FluidSet::GetBuoyancy()
{
	return this->buoyancy;
}

//////////////////////////////////////////////////
void FluidSet::SetSmoothingLength(float _length)
{
	this->smoothingLength = _length;
}

//////////////////////////////////////////////////
float FluidSet::GetSmoothingLength()
{
	return this->smoothingLength;
}

//////////////////////////////////////////////////
void FluidSet::SetSmoothingNeighborsNr(int _nr)
{
	this->smoothingNeighboursNr = _nr;
}

//////////////////////////////////////////////////
int FluidSet::GetSmoothingNeighboursNr()
{
	return this->smoothingNeighboursNr;
}



