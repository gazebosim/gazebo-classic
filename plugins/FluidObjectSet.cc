/* Desc: Object Set class
 * Author: Andrei Haidu
 * Date: 11 Jul. 2013
 */

#include "FluidObjectSet.hh"
#include "vector_functions.h"

using namespace fluidix;

//////////////////////////////////////////////////
ObjectSet::ObjectSet()
{
}

//////////////////////////////////////////////////
ObjectSet::~ObjectSet()
{

}

//////////////////////////////////////////////////
void ObjectSet::Load()
{

}

//////////////////////////////////////////////////
void ObjectSet::Fini()
{

}

//////////////////////////////////////////////////
void ObjectSet::Init()
{

}

//////////////////////////////////////////////////
void ObjectSet::Reset()
{

}

//////////////////////////////////////////////////
void ObjectSet::Update()
{

}


//////////////////////////////////////////////////
void ObjectSet::SetParticleSetId(int _id)
{
	this->particleSetId = _id;
}

//////////////////////////////////////////////////
int ObjectSet::GetParticleSetId()
{
	return this->particleSetId;
}

//////////////////////////////////////////////////
int ObjectSet::GetLinkId()
{
    return this->linkSetId;
}

//////////////////////////////////////////////////
void ObjectSet::SetLinkSetId(int _linkSetId)
{
    this->linkSetId = _linkSetId;
}

//////////////////////////////////////////////////
void ObjectSet::SetParticleAndLinkSetId(int _particleSetId, int _linkSetId)
{
    this->particleSetId = _particleSetId;

    this->linkSetId = _linkSetId;
}

//////////////////////////////////////////////////
void ObjectSet::SetParticleAndLinkSetId(int2 _particleLinkSetId)
{
    // .x = particle set index , .y link set index
    this->particleSetId = _particleLinkSetId.x;

    this->linkSetId = _particleLinkSetId.y;
}

//////////////////////////////////////////////////
void ObjectSet::SetRestitutionCoef(float _coef)
{
    this->restitutionCoef = _coef;
}

//////////////////////////////////////////////////
float ObjectSet::GetRestitutionCoef()
{
    return this->restitutionCoef;
}

//////////////////////////////////////////////////
void ObjectSet::SetCollisionForceSum(float4 _sum_coll_force)
{
    this->collisionForceSum = _sum_coll_force;
}

//////////////////////////////////////////////////
float4 ObjectSet::GetCollisionForceSum()
{
    return this->collisionForceSum;
}

//////////////////////////////////////////////////
void ObjectSet::SetCollisionForcePosSum(float4 _sum_coll_force)
{
    this->collisionForcePosSum = _sum_coll_force;
}

//////////////////////////////////////////////////
float4 ObjectSet::GetCollisionForcePosSum()
{
    return this->collisionForcePosSum;
}

//////////////////////////////////////////////////
void ObjectSet::SetWorldPosition(float4 _worldPosition)
{
    this->worldPosition = _worldPosition;
}

//////////////////////////////////////////////////
void ObjectSet::SetWorldPosition(float3 _worldPos3)
{
    this->worldPosition.x = _worldPos3.x;
    this->worldPosition.y = _worldPos3.y;
    this->worldPosition.z = _worldPos3.z;
}

//////////////////////////////////////////////////
void ObjectSet::SetWorldPosition(float _x, float _y, float _z)
{
    this->worldPosition.x = _x;
    this->worldPosition.y = _y;
    this->worldPosition.z = _z;
}

//////////////////////////////////////////////////
float4 ObjectSet::GetWorldPosition()
{
    return this->worldPosition;
}

//////////////////////////////////////////////////
void ObjectSet::SetWorldOrientation(Quaternion _quat)
{
    this->worldOrientation = _quat;
}

//////////////////////////////////////////////////
Quaternion ObjectSet::GetWorldOrientation()
{
    return this->worldOrientation;
}

