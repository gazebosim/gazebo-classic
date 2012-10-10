
/*
-----------------------------------------------------------------------------
This source file is part of GIMPACT Library.

For the latest info, see http://gimpact.sourceforge.net/

Copyright (c) 2006 Francisco Leon. C.C. 80087371.
email: projectileman@yahoo.com

 This library is free software; you can redistribute it and/or
 modify it under the terms of EITHER:
   (1) The GNU Lesser General Public License as published by the Free
       Software Foundation; either version 2.1 of the License, or (at
       your option) any later version. The text of the GNU Lesser
       General Public License is included with this library in the
       file GIMPACT-LICENSE-LGPL.TXT.
   (2) The BSD-style license that is included with this library in
       the file GIMPACT-LICENSE-BSD.TXT.

 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files
 GIMPACT-LICENSE-LGPL.TXT and GIMPACT-LICENSE-BSD.TXT for more details.

-----------------------------------------------------------------------------
*/

#include "GIMPACT/gim_trimesh.h"


//! Trimesh Ray Collisions
/*!

\param trimesh
\param contact
\return 1 if the ray collides, else 0
*/
int gim_trimesh_ray_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact)
{
    GDYNAMIC_ARRAY collision_result;
	GIM_CREATE_BOXQUERY_LIST(collision_result);

	gim_aabbset_ray_collision(origin,dir,tmax,&trimesh->m_aabbset,&collision_result);

	if(collision_result.m_size==0)
	{
	    GIM_DYNARRAY_DESTROY(collision_result);
	    return 0;
	}

	//collide triangles

	GUINT32 * boxesresult = GIM_DYNARRAY_POINTER(GUINT32,collision_result);
	GIM_TRIANGLE_DATA  tridata;
	vec3f pout;
	GREAL tparam,u,v;
	char does_intersect;

	gim_trimesh_locks_work_data(trimesh);

	for(unsigned int i=0;i<collision_result.m_size;i++)
	{
		gim_trimesh_get_triangle_data(trimesh,boxesresult[i],&tridata);
        
		// flip plane for correct result in ODE
		// for more info: martijn@bytehazard.com
		vec4f flippedPlane;
		VEC_SCALE_4(flippedPlane, -1.0f, tridata.m_planes.m_planes[0]);
        
		RAY_TRIANGLE_INTERSECTION(origin,dir,tridata.m_vertices[0],tridata.m_vertices[1],tridata.m_vertices[2],flippedPlane,pout,u,v,tparam,tmax,does_intersect);
		if(does_intersect)
		{
		    contact->tparam = tparam;
		    contact->u = u;
		    contact->v = v;
		    contact->m_face_id = boxesresult[i];
		    VEC_COPY(contact->m_point,pout);
		    VEC_COPY(contact->m_normal,flippedPlane);

		    gim_trimesh_unlocks_work_data(trimesh);
            GIM_DYNARRAY_DESTROY(collision_result);
		    return 1;
		}
	}

	gim_trimesh_unlocks_work_data(trimesh);
	GIM_DYNARRAY_DESTROY(collision_result);
	return 0;//no collisiion
}


//! Trimesh Ray Collisions closest
/*!
Find the closest primitive collided by the ray
\param trimesh
\param contact
\return 1 if the ray collides, else 0
*/
int gim_trimesh_ray_closest_collision(GIM_TRIMESH * trimesh,vec3f origin,vec3f dir, GREAL tmax, GIM_TRIANGLE_RAY_CONTACT_DATA * contact)
{
    GDYNAMIC_ARRAY collision_result;
	GIM_CREATE_BOXQUERY_LIST(collision_result);

	gim_aabbset_ray_collision(origin,dir,tmax,&trimesh->m_aabbset,&collision_result);

	if(collision_result.m_size==0)
	{
	    GIM_DYNARRAY_DESTROY(collision_result);
	    return 0;
	}

	//collide triangles

	GUINT32 * boxesresult = GIM_DYNARRAY_POINTER(GUINT32,collision_result);
	GIM_TRIANGLE_DATA  tridata;
	vec3f pout;
	GREAL tparam,u,v;
	char does_intersect;
	contact->tparam = tmax + 0.1f;

	gim_trimesh_locks_work_data(trimesh);

	for(unsigned int i=0;i<collision_result.m_size;i++)
	{
		gim_trimesh_get_triangle_data(trimesh,boxesresult[i],&tridata);

		// flip plane for correct result in ODE
		// for more info: martijn@bytehazard.com
		vec4f flippedPlane;
		VEC_SCALE_4(flippedPlane, -1.0f, tridata.m_planes.m_planes[0]);

		RAY_TRIANGLE_INTERSECTION(origin,dir,tridata.m_vertices[0],tridata.m_vertices[1],tridata.m_vertices[2],flippedPlane,pout,u,v,tparam,tmax,does_intersect);
		if(does_intersect && (tparam < contact->tparam))
		{
            contact->tparam = tparam;
		    contact->u = u;
		    contact->v = v;
		    contact->m_face_id = boxesresult[i];
		    VEC_COPY(contact->m_point,pout);
		    VEC_COPY(contact->m_normal,flippedPlane);
		}
	}

	gim_trimesh_unlocks_work_data(trimesh);
	GIM_DYNARRAY_DESTROY(collision_result);
	if(contact->tparam > tmax) return 0;
	return 1;
}
