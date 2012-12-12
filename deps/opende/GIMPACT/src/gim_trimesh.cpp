
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


#include <assert.h>
#include <iostream>
#include "GIMPACT/gim_trimesh.h"

GUINT32 gim_trimesh_get_triangle_count(GIM_TRIMESH * trimesh)
{
    return trimesh->m_tri_index_buffer.m_element_count/3;
}

//! Creates the aabb set and the triangles cache
/*!

\param trimesh
\param vertex_array
\param triindex_array
\param transformed_reply If 1, then the m_transformed_vertices is a reply of the source vertices. Else it just be a reference to the original array.
\post it copies the arrays by reference, and creates the auxiliary data (m_aabbset,m_planes_cache_buffer)
*/
void gim_trimesh_create_from_arrays(GBUFFER_MANAGER_DATA buffer_managers[],
  GIM_TRIMESH * trimesh, GBUFFER_ARRAY * vertex_array, GBUFFER_ARRAY * triindex_array,char transformed_reply)
{
    assert(trimesh);
    assert(vertex_array);
    assert(triindex_array);
    gim_buffer_array_copy_ref(vertex_array,&trimesh->m_source_vertex_buffer);
    gim_buffer_array_copy_ref(triindex_array,&trimesh->m_tri_index_buffer);

    trimesh->m_mask = GIM_TRIMESH_NEED_UPDATE;//needs update
    //Create the transformed vertices
    if(transformed_reply==1)
    {
        trimesh->m_mask |= GIM_TRIMESH_TRANSFORMED_REPLY;
        gim_buffer_array_copy_value(vertex_array,
      buffer_managers,&trimesh->m_transformed_vertex_buffer,G_BUFFER_MANAGER_SYSTEM,G_MU_DYNAMIC_READ_WRITE);
    }
    else
    {
        gim_buffer_array_copy_ref(vertex_array,&trimesh->m_transformed_vertex_buffer);
    }
    //create the box set
    GUINT32 facecount = gim_trimesh_get_triangle_count(trimesh);

    gim_aabbset_alloc(&trimesh->m_aabbset,facecount);
    //create the planes cache
    GIM_DYNARRAY_CREATE_SIZED(GIM_TRIPLANES_CACHE,trimesh->m_planes_cache_buffer,facecount);
    //Create the bitset
    GIM_BITSET_CREATE_SIZED(trimesh->m_planes_cache_bitset,facecount);
    //Callback is 0
    trimesh->m_update_callback = 0;
    //set to identity
    IDENTIFY_MATRIX_4X4(trimesh->m_transform);
}



//! Create a trimesh from vertex array and an index array
/*!

\param trimesh An uninitialized GIM_TRIMESH  structure
\param vertex_array A buffer to a vec3f array
\param vertex_count
\param triindex_array
\param index_count
\param copy_vertices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param copy_indices If 1, it copies the source vertices in another buffer. Else (0) it constructs a reference to the data.
\param transformed_reply If , then the m_transformed_vertices is a reply of the source vertices. Else it just be a reference to the original array.
*/
void gim_trimesh_create_from_data(GBUFFER_MANAGER_DATA buffer_managers[],
  GIM_TRIMESH * trimesh, vec3f * vertex_array, GUINT32 vertex_count,char copy_vertices,
  GUINT32 * triindex_array, GUINT32 index_count,char copy_indices,char transformed_reply)
{
    GBUFFER_ARRAY buffer_vertex_array;
    GBUFFER_ARRAY buffer_triindex_array;

    //Create vertices
    if(copy_vertices == 1)
    {
        gim_create_common_buffer_from_data(buffer_managers,
      vertex_array, vertex_count*sizeof(vec3f), &buffer_vertex_array.m_buffer_id);
    }
    else//Create a shared buffer
    {
        gim_create_shared_buffer_from_data(buffer_managers,
      vertex_array, vertex_count*sizeof(vec3f), &buffer_vertex_array.m_buffer_id);
    }
    GIM_BUFFER_ARRAY_INIT_TYPE(vec3f,buffer_vertex_array,buffer_vertex_array.m_buffer_id,vertex_count);


    //Create vertices
    if(copy_indices == 1)
    {
        gim_create_common_buffer_from_data(buffer_managers,
      triindex_array, index_count*sizeof(GUINT32), &buffer_triindex_array.m_buffer_id);
    }
    else//Create a shared buffer
    {
        gim_create_shared_buffer_from_data(buffer_managers,
      triindex_array, index_count*sizeof(GUINT32), &buffer_triindex_array.m_buffer_id);
    }
    GIM_BUFFER_ARRAY_INIT_TYPE(GUINT32,buffer_triindex_array,buffer_triindex_array.m_buffer_id,index_count);

    gim_trimesh_create_from_arrays(buffer_managers, trimesh,
    &buffer_vertex_array, &buffer_triindex_array,transformed_reply);

    ///always call this after create a buffer_array
    GIM_BUFFER_ARRAY_DESTROY(buffer_vertex_array);
    GIM_BUFFER_ARRAY_DESTROY(buffer_triindex_array);
}

//! Clears auxiliary data and releases buffer arrays
void gim_trimesh_destroy(GIM_TRIMESH * trimesh)
{
    gim_aabbset_destroy(&trimesh->m_aabbset);

    GIM_DYNARRAY_DESTROY(trimesh->m_planes_cache_buffer);
    GIM_DYNARRAY_DESTROY(trimesh->m_planes_cache_bitset);

    GIM_BUFFER_ARRAY_DESTROY(trimesh->m_transformed_vertex_buffer);
    GIM_BUFFER_ARRAY_DESTROY(trimesh->m_source_vertex_buffer);
    GIM_BUFFER_ARRAY_DESTROY(trimesh->m_tri_index_buffer);
}

//! Copies two meshes
/*!
\pre dest_trimesh shouldn't be created
\post dest_trimesh will be created
\param source_trimesh
\param dest_trimesh
\param copy_by_reference If 1, it attach a reference to the source vertices, else it copies the vertices
\param transformed_reply IF 1, then it forces the m_trasnformed_vertices to be  a reply of the source vertices
*/
void gim_trimesh_copy(GIM_TRIMESH * source_trimesh,
  GBUFFER_MANAGER_DATA dest_buffer_managers[], GIM_TRIMESH * dest_trimesh,
  char /*copy_by_reference*/, char transformed_reply)
{
/* -- trimesh can not be copied by reference until GBUFFER_MANAGER_DATA is rewritten
  to be thread safe and until it is moved back to global variables.
    if(copy_by_reference==1)
    {
        gim_trimesh_create_from_arrays(dest_trimesh, &source_trimesh->m_source_vertex_buffer, &source_trimesh->m_tri_index_buffer,transformed_reply);
    }
    else
*/
    {
        GBUFFER_ARRAY buffer_vertex_array;
        GBUFFER_ARRAY buffer_triindex_array;

        gim_buffer_array_copy_value(&source_trimesh->m_source_vertex_buffer,
      dest_buffer_managers,&buffer_vertex_array,G_BUFFER_MANAGER_SYSTEM,G_MU_DYNAMIC_READ_WRITE);

        gim_buffer_array_copy_value(&source_trimesh->m_tri_index_buffer,
      dest_buffer_managers,&buffer_triindex_array,G_BUFFER_MANAGER_SYSTEM,G_MU_DYNAMIC_READ_WRITE);

        gim_trimesh_create_from_arrays(dest_buffer_managers, dest_trimesh,
      &buffer_vertex_array, &buffer_triindex_array,transformed_reply);

        ///always call this after create a buffer_array
        GIM_BUFFER_ARRAY_DESTROY(buffer_vertex_array);
        GIM_BUFFER_ARRAY_DESTROY(buffer_triindex_array);
    }
}
//! Locks the trimesh for working with it
/*!
\post locks m_tri_index_buffer and m_transformed_vertex_buffer.
\param trimesh
*/
void gim_trimesh_locks_work_data(GIM_TRIMESH * trimesh)
{
    GINT32 res = 0;
    res=gim_buffer_array_lock(&trimesh->m_tri_index_buffer,G_MA_READ_ONLY);
    if (res != G_BUFFER_OP_SUCCESS)
      std::cerr << "Inavalid lock\n";
    assert(res==G_BUFFER_OP_SUCCESS);
    res=gim_buffer_array_lock(&trimesh->m_transformed_vertex_buffer,G_MA_READ_ONLY);
    if (res != G_BUFFER_OP_SUCCESS)
      std::cerr << "Inavalid lock\n";
    assert(res==G_BUFFER_OP_SUCCESS);
}

//! unlocks the trimesh
/*!
\post unlocks m_tri_index_buffer and m_transformed_vertex_buffer.
\param trimesh
*/
void gim_trimesh_unlocks_work_data(GIM_TRIMESH * trimesh)
{
    gim_buffer_array_unlock(&trimesh->m_tri_index_buffer);
    gim_buffer_array_unlock(&trimesh->m_transformed_vertex_buffer);
}


//! Returns 1 if the m_transformed_vertex_buffer is a reply of m_source_vertex_buffer
char gim_trimesh_has_tranformed_reply(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_mask&GIM_TRIMESH_TRANSFORMED_REPLY) return 1;
    return 0;
}

//! Returns 1 if the trimesh needs to update their aabbset and the planes cache.
char gim_trimesh_needs_update(GIM_TRIMESH * trimesh)
{
    if(trimesh->m_mask&GIM_TRIMESH_NEED_UPDATE) return 1;
    return 0;
}

//! Change the state of the trimesh for force it to update
/*!
Call it after made changes to the trimesh.
\post gim_trimesh_need_update(trimesh) will return 1
*/
void gim_trimesh_post_update(GIM_TRIMESH * trimesh)
{
    trimesh->m_mask |= GIM_TRIMESH_NEED_UPDATE;
}

//kernel
#define MULT_MAT_VEC4_KERNEL(_mat,_src,_dst) MAT_DOT_VEC_3X4((_dst),(_mat),(_src))

//! Updates m_transformed_vertex_buffer
/*!
\pre m_transformed_vertex_buffer must be unlocked
*/
void gim_trimesh_update_vertices(GIM_TRIMESH * trimesh)
{
    if(gim_trimesh_has_tranformed_reply(trimesh) == 0) return; //Don't perform transformation

    //Vertices
    GBUFFER_ARRAY * psource_vertex_buffer = &trimesh->m_source_vertex_buffer;
    GBUFFER_ARRAY * ptransformed_vertex_buffer = &trimesh->m_transformed_vertex_buffer;
    //Temp transform
    mat4f transform;
    COPY_MATRIX_4X4(transform,trimesh->m_transform);

    GIM_PROCESS_BUFFER_ARRAY(transform,(*psource_vertex_buffer),(*ptransformed_vertex_buffer),MULT_MAT_VEC4_KERNEL,vec3f,vec3f);
}

//! Updates m_aabbset and m_planes_cache_bitset
/*!
\pre gim_trimesh_locks_work_data must be called before
*/
void gim_trimesh_update_aabbset(GIM_TRIMESH * trimesh)
{
    vec3f * transformed_vertices = GIM_BUFFER_ARRAY_POINTER(vec3f,trimesh->m_transformed_vertex_buffer,0);
    assert(transformed_vertices);

    GUINT32 * triangle_indices = GIM_BUFFER_ARRAY_POINTER(GUINT32,trimesh->m_tri_index_buffer,0);
    assert(triangle_indices);
    // box set
    aabb3f * paabb = trimesh->m_aabbset.m_boxes;
    GUINT32 triangle_count = gim_trimesh_get_triangle_count(trimesh);
    float * v1,*v2,*v3;
    GUINT32 i;
    for (i=0; i<triangle_count;i++)
    {
        v1 = &transformed_vertices[triangle_indices[0]][0];
        v2 = &transformed_vertices[triangle_indices[1]][0];
        v3 = &transformed_vertices[triangle_indices[2]][0];
        COMPUTEAABB_FOR_TRIANGLE((*paabb),v1,v2,v3);
        triangle_indices+=3;
        paabb++;
    }
    //Clear planes cache
    GIM_BITSET_CLEAR_ALL(trimesh->m_planes_cache_bitset);
    //Sorts set
    gim_aabbset_update(&trimesh->m_aabbset);
}

//! Updates the trimesh if needed
/*!
\post If gim_trimesh_needs_update returns 1, then it calls  gim_trimesh_update_vertices and gim_trimesh_update_aabbset
*/
void gim_trimesh_update(GIM_TRIMESH * trimesh)
{
    if(gim_trimesh_needs_update(trimesh)==0) return;
    gim_trimesh_update_vertices(trimesh);
    gim_trimesh_locks_work_data(trimesh);
    gim_trimesh_update_aabbset(trimesh);
    gim_trimesh_unlocks_work_data(trimesh);

    //Clear update flag
     trimesh->m_mask &= ~GIM_TRIMESH_NEED_UPDATE;
}

void gim_trimesh_set_tranform(GIM_TRIMESH * trimesh, mat4f transform)
{
    GREAL diff = 0.0f;
    float * originaltrans = &trimesh->m_transform[0][0];
    float * newtrans = &transform[0][0];
    GUINT32 i;
    for (i=0;i<16;i++)
    {
      diff += fabs(originaltrans[i]-newtrans[i]);
    }

//    if(IS_ZERO(diff)) return ;///don't need to update
    if(diff< 0.00001f) return ;///don't need to update

    COPY_MATRIX_4X4(trimesh->m_transform,transform);

    gim_trimesh_post_update(trimesh);
}

void gim_trimesh_get_triangle_data(GIM_TRIMESH * trimesh, GUINT32 triangle_index, GIM_TRIANGLE_DATA * tri_data)
{
    vec3f * transformed_vertices = GIM_BUFFER_ARRAY_POINTER(vec3f,trimesh->m_transformed_vertex_buffer,0);

    GUINT32 * triangle_indices = GIM_BUFFER_ARRAY_POINTER(GUINT32,trimesh->m_tri_index_buffer,triangle_index*3);


    //Copy the vertices
    VEC_COPY(tri_data->m_vertices[0],transformed_vertices[triangle_indices[0]]);
    VEC_COPY(tri_data->m_vertices[1],transformed_vertices[triangle_indices[1]]);
    VEC_COPY(tri_data->m_vertices[2],transformed_vertices[triangle_indices[2]]);

    //Get the planes
    GIM_TRIPLANES_CACHE * planes = GIM_DYNARRAY_POINTER(GIM_TRIPLANES_CACHE,trimesh->m_planes_cache_buffer);
    planes += triangle_index;

    //verify planes cache
    GUINT32 bit_eval;
    GIM_BITSET_GET(trimesh->m_planes_cache_bitset,triangle_index,bit_eval);
    if(bit_eval == 0)// Needs to calc the planes
    {
        //Calc the face plane
        TRIANGLE_PLANE(tri_data->m_vertices[0],tri_data->m_vertices[1],tri_data->m_vertices[2],planes->m_planes[0]);
        //Calc the edge 1
        EDGE_PLANE(tri_data->m_vertices[0],tri_data->m_vertices[1],(planes->m_planes[0]),(planes->m_planes[1]));

        //Calc the edge 2
        EDGE_PLANE(tri_data->m_vertices[1],tri_data->m_vertices[2],(planes->m_planes[0]),(planes->m_planes[2]));

        //Calc the edge 3
        EDGE_PLANE(tri_data->m_vertices[2],tri_data->m_vertices[0],(planes->m_planes[0]), (planes->m_planes[3]));

        //mark
        GIM_BITSET_SET(trimesh->m_planes_cache_bitset,triangle_index);
    }


    VEC_COPY_4((tri_data->m_planes.m_planes[0]),(planes->m_planes[0]));//face plane
    VEC_COPY_4((tri_data->m_planes.m_planes[1]),(planes->m_planes[1]));//edge1
    VEC_COPY_4((tri_data->m_planes.m_planes[2]),(planes->m_planes[2]));//edge2
    VEC_COPY_4((tri_data->m_planes.m_planes[3]),(planes->m_planes[3]));//edge3
}

void gim_trimesh_get_triangle_vertices(GIM_TRIMESH * trimesh, GUINT32 triangle_index, vec3f v1,vec3f v2,vec3f v3)
{
    vec3f * transformed_vertices = GIM_BUFFER_ARRAY_POINTER(vec3f,trimesh->m_transformed_vertex_buffer,0);

    GUINT32 * triangle_indices = GIM_BUFFER_ARRAY_POINTER(GUINT32,trimesh->m_tri_index_buffer,triangle_index*3);

    //Copy the vertices
    VEC_COPY(v1,transformed_vertices[triangle_indices[0]]);
    VEC_COPY(v2,transformed_vertices[triangle_indices[1]]);
    VEC_COPY(v3,transformed_vertices[triangle_indices[2]]);
}
