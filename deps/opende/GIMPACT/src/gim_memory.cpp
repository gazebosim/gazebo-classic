
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
#include <stdlib.h>
#include "GIMPACT/gim_memory.h"
#include <gazebo/ode/odeconfig.h>
#include "config.h"
//#include "malloc.h"
//#include "mm_malloc.h"

static gim_alloc_function *g_allocfn = 0;
// static gim_alloca_function *g_allocafn = 0; -- a nonsense
static gim_realloc_function *g_reallocfn = 0;
static gim_free_function *g_freefn = 0;


#define VALIDATE_BUFFER_MANAGER(buffer_managers,buffer_manager_id)\
    if(buffer_manager_id>=G_BUFFER_MANAGER__MAX) return G_BUFFER_OP_INVALID;\
    GBUFFER_MANAGER_DATA * bm_data;\
    gim_get_buffer_manager_data(buffer_managers,buffer_manager_id,&bm_data);\
    if(bm_data == 0) return G_BUFFER_OP_INVALID;\

#define VALIDATE_BUFFER_ID_PT(buffer_id)\
  GBUFFER_MANAGER_DATA * bm_data = buffer_id->m_bm_data;\
  if(bm_data == 0) return G_BUFFER_OP_INVALID;\
    if(buffer_id->m_buffer_id>=bm_data->m_buffer_array.m_size) return G_BUFFER_OP_INVALID;\
    GBUFFER_DATA * pbuffer = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);\
    pbuffer += buffer_id->m_buffer_id;\
    if(pbuffer->m_buffer_handle==0) return G_BUFFER_OP_INVALID;\


void GIM_BUFFER_ARRAY_DESTROY(GBUFFER_ARRAY & array_data)
{
    gim_buffer_array_unlock(&array_data);
    gim_buffer_free(&(array_data).m_buffer_id);
}

void GIM_DYNARRAY_DESTROY(GDYNAMIC_ARRAY & array_data)
{
    if(array_data.m_pdata != 0)
    {
        gim_free(array_data.m_pdata,0);
        array_data.m_reserve_size = 0;
        array_data.m_size = 0;
        array_data.m_pdata = 0;
    }
}

void gim_set_alloc_handler (gim_alloc_function *fn)
{
  g_allocfn = fn;
}

/* -- a nonsense
void gim_set_alloca_handler (gim_alloca_function *fn)
{
  g_allocafn = fn;
}
*/

void gim_set_realloc_handler (gim_realloc_function *fn)
{
  g_reallocfn = fn;
}

void gim_set_free_handler (gim_free_function *fn)
{
  g_freefn = fn;
}

gim_alloc_function *gim_get_alloc_handler()
{
  return g_allocfn;
}

/* -- a nonsense
gim_alloca_function *gim_get_alloca_handler()
{
  return g_allocafn;
}
*/

gim_realloc_function *gim_get_realloc_handler ()
{
  return g_reallocfn;
}


gim_free_function  *gim_get_free_handler ()
{
  return g_freefn;
}


void * gim_alloc(size_t size)
{
  void * ptr;
/*
  if (g_allocfn)
  {
    ptr = g_allocfn(size);
  }
  else
*/
  {
    ptr = malloc(size);//_mm_malloc(size,0);*/
  }
  assert(ptr);
  return ptr;
}

/* -- a nonsense
void * gim_alloca(size_t size)
{
  if (g_allocafn) return g_allocafn(size); else return alloca(size);
}
*/

void * gim_realloc(void *ptr, size_t oldsize, size_t newsize)
{
  /*if (g_reallocfn) return g_reallocfn(ptr,oldsize,newsize);
  else return realloc(ptr,newsize);*/
  //return realloc(ptr,newsize);
  void * newptr = gim_alloc(newsize);
  size_t copysize = newsize> oldsize? oldsize: newsize;
  memcpy(newptr,ptr,copysize);
  gim_free(ptr,oldsize);
  return newptr;
}

void gim_free(void *ptr, size_t /*size*/)
{
  if (!ptr) return;
/* -- if custom allocation function is not used, custom free must not be used too
  if (g_freefn)
  {
       g_freefn(ptr,size);
  }
  else
*/
  {
      free(ptr);//_mm_free(ptr);
  }
}

///******************************* BUFFER MANAGERS ******************************///

//!** Basic buffer prototype functions

static GPTR _system_buffer_alloc_function(GUINT32 size,int /*usage*/)
{
    void * newdata = gim_alloc(size);
    memset(newdata,0,size);
    return (GPTR)newdata;
}

static GPTR _system_buffer_alloc_data_function(const void * pdata,GUINT32 size,int /*usage*/)
{
    void * newdata = gim_alloc(size);
    memcpy(newdata,pdata,size);
    return (GPTR)(newdata);
}

static GPTR _system_buffer_realloc_function(GPTR buffer_handle,GUINT32 oldsize,int /*old_usage*/,GUINT32 newsize,int /*new_usage*/)
{
    void * newdata = gim_realloc(buffer_handle,oldsize,newsize);
    return (GPTR)(newdata);
}

static void _system_buffer_free_function(GPTR buffer_handle,GUINT32 size)
{
    gim_free(buffer_handle,size);
}

static char * _system_lock_buffer_function(GPTR buffer_handle,int /*access*/)
{
    return (char * )(buffer_handle);
}


static void _system_unlock_buffer_function(GPTR /*buffer_handle*/)
{
}

static void _system_download_from_buffer_function(
        GPTR source_buffer_handle,
    GUINT32 source_pos,
    void * destdata,
    GUINT32 copysize)
{
    char * pdata;
  pdata = (char *)source_buffer_handle;
  memcpy(destdata,pdata+source_pos,copysize);
}

static void  _system_upload_to_buffer_function(
        GPTR dest_buffer_handle,
    GUINT32 dest_pos,
    void * sourcedata,
    GUINT32 copysize)
{
    char * pdata;
  pdata = (char * )dest_buffer_handle;
  memcpy(pdata+dest_pos,sourcedata,copysize);
}

static void  _system_copy_buffers_function(
    GPTR source_buffer_handle,
    GUINT32 source_pos,
    GPTR dest_buffer_handle,
    GUINT32 dest_pos,
    GUINT32 copysize)
{
    char * pdata1,*pdata2;
  pdata1 = (char *)source_buffer_handle;
  pdata2 = (char *)dest_buffer_handle;
  memcpy(pdata2+dest_pos,pdata1+source_pos,copysize);
}

static GPTR _shared_buffer_alloc_function(GUINT32 /*size*/,int /*usage*/)
{
    return 0;
}

static GPTR _shared_buffer_alloc_data_function(const void * pdata,GUINT32 /*size*/,int /*usage*/)
{
    return (GPTR)pdata;
}

// static GPTR _shared_buffer_realloc_function(GPTR /*buffer_handle*/,GUINT32 /*oldsize*/,int /*old_usage*/,GUINT32 /*newsize*/,int /*new_usage*/)
//{
//    return 0;
//}

static void _shared_buffer_free_function(GPTR /*buffer_handle*/,GUINT32 /*size*/)
{
}

static inline int _is_buffer_manager_data_active(GBUFFER_MANAGER_DATA * bm_data)
{
  return bm_data->m_buffer_array.m_pdata != 0;
}

static inline void _init_buffer_manager_data(GBUFFER_MANAGER_DATA * bm_data)
{
  bm_data->m_buffer_array.m_pdata = 0;
}

static const GBUFFER_MANAGER_PROTOTYPE g_bm_prototypes[G_BUFFER_MANAGER__MAX] =
{
  {
    &_system_buffer_alloc_function, // alloc_fn;
    &_system_buffer_alloc_data_function, // alloc_data_fn;
    &_system_buffer_realloc_function, // realloc_fn;
    &_system_buffer_free_function, // free_fn;
    &_system_lock_buffer_function, // lock_buffer_fn;
    &_system_unlock_buffer_function, // unlock_buffer_fn;
    &_system_download_from_buffer_function, // download_from_buffer_fn;
    &_system_upload_to_buffer_function, // upload_to_buffer_fn;
    &_system_copy_buffers_function, // copy_buffers_fn;
  }, // G_BUFFER_MANAGER_SYSTEM

  {
    &_shared_buffer_alloc_function, // alloc_fn;
    &_shared_buffer_alloc_data_function, // alloc_data_fn;
    &_system_buffer_realloc_function, // realloc_fn;
    &_shared_buffer_free_function, // free_fn;
    &_system_lock_buffer_function, // lock_buffer_fn;
    &_system_unlock_buffer_function, // unlock_buffer_fn;
    &_system_download_from_buffer_function, // download_from_buffer_fn;
    &_system_upload_to_buffer_function, // upload_to_buffer_fn;
    &_system_copy_buffers_function, // copy_buffers_fn;
  }, // G_BUFFER_MANAGER_SHARED
};

int gim_is_buffer_manager_active(GBUFFER_MANAGER_DATA buffer_managers[],
  GUINT32 buffer_manager_id)
{
  GBUFFER_MANAGER_DATA * bm_data;
  bm_data = &buffer_managers[buffer_manager_id];
  return _is_buffer_manager_data_active(bm_data);
}

//!** Buffer manager operations
void gim_create_buffer_manager(GBUFFER_MANAGER_DATA buffer_managers[],
  GUINT32 buffer_manager_id)
{
    GBUFFER_MANAGER_DATA * bm_data;
    bm_data = &buffer_managers[buffer_manager_id];

  if (_is_buffer_manager_data_active(bm_data))
  {
    gim_destroy_buffer_manager(buffer_managers, buffer_manager_id);
  }

    //CREATE ARRAYS
    GIM_DYNARRAY_CREATE(GBUFFER_DATA,bm_data->m_buffer_array,G_ARRAY_BUFFERMANAGER_INIT_SIZE);
    GIM_DYNARRAY_CREATE(GUINT32,bm_data->m_free_positions,G_ARRAY_BUFFERMANAGER_INIT_SIZE);
  bm_data->m_prototype = g_bm_prototypes + buffer_manager_id;
  bm_data->m_buffer_manager_id = buffer_manager_id;
}

void gim_destroy_buffer_manager(GBUFFER_MANAGER_DATA buffer_managers[], GUINT32 buffer_manager_id)
{
    GBUFFER_MANAGER_DATA * bm_data;
    gim_get_buffer_manager_data(buffer_managers,buffer_manager_id,&bm_data);
    if(bm_data == 0) return;
    //Destroy all buffers

    GBUFFER_DATA * buffers = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);
    GUINT32 i, buffer_count = bm_data->m_buffer_array.m_size;
    for (i=0;i<buffer_count ;i++ )
    {
    GBUFFER_DATA * current_buffer = buffers + i;
      if(current_buffer->m_buffer_handle!=0) //Is active
      {
          // free handle
          bm_data->m_prototype->free_fn(current_buffer->m_buffer_handle,current_buffer->m_size);
      }
    }

    //destroy buffer array
    GIM_DYNARRAY_DESTROY(bm_data->m_buffer_array);
    //destroy free positions
    GIM_DYNARRAY_DESTROY(bm_data->m_free_positions);
}
void gim_get_buffer_manager_data(GBUFFER_MANAGER_DATA buffer_managers[],
  GUINT32 buffer_manager_id,GBUFFER_MANAGER_DATA ** pbm_data)
{
  GBUFFER_MANAGER_DATA * bm_data;
    bm_data = &buffer_managers[buffer_manager_id];

    if (_is_buffer_manager_data_active(bm_data))
    {
    *pbm_data = bm_data;
    }
    else
    {
    *pbm_data = 0;
    }
}

void gim_init_buffer_managers(GBUFFER_MANAGER_DATA buffer_managers[])
{
    GUINT32 i;
    for (i=0;i<G_BUFFER_MANAGER__MAX;i++)
    {
    _init_buffer_manager_data(buffer_managers + i);
    }

  // Add the two most important buffer managers

    //add system buffer manager
    gim_create_buffer_manager(buffer_managers,G_BUFFER_MANAGER_SYSTEM );

    //add shared buffer manager
    gim_create_buffer_manager(buffer_managers,G_BUFFER_MANAGER_SHARED);
}

void gim_terminate_buffer_managers(GBUFFER_MANAGER_DATA buffer_managers[])
{
    GUINT32 i;
    for (i=0;i<G_BUFFER_MANAGER__MAX;i++)
    {
        gim_destroy_buffer_manager(buffer_managers,i);
    }
}

//!** Buffer operations

void GET_AVALIABLE_BUFFER_ID(GBUFFER_MANAGER_DATA * buffer_manager, GUINT32 & buffer_id)
{
    if(buffer_manager->m_free_positions.m_size>0)\
    {
        GUINT32 * _pointer = GIM_DYNARRAY_POINTER(GUINT32,buffer_manager->m_free_positions);
        buffer_id = _pointer[buffer_manager->m_free_positions.m_size-1];
        GIM_DYNARRAY_POP_ITEM(buffer_manager->m_free_positions);
    }
    else
    {
        buffer_id = buffer_manager->m_buffer_array.m_size;
        GIM_DYNARRAY_PUSH_EMPTY(GBUFFER_DATA,buffer_manager->m_buffer_array);
    }
}

GINT32 _validate_buffer_id(GBUFFER_ID * buffer_id,GBUFFER_DATA ** ppbuffer,GBUFFER_MANAGER_DATA ** pbm_data)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    *ppbuffer = pbuffer;
    *pbm_data = bm_data;
    return G_BUFFER_OP_SUCCESS;
}

GUINT32 gim_create_buffer(
  GBUFFER_MANAGER_DATA buffer_managers[],
    GUINT32 buffer_manager_id,
    GUINT32 buffer_size,
    int usage,
    GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_MANAGER(buffer_managers,buffer_manager_id)

    GPTR newbufferhandle = bm_data->m_prototype->alloc_fn(buffer_size,usage);
    if(newbufferhandle==0) return G_BUFFER_OP_INVALID;

    GET_AVALIABLE_BUFFER_ID(bm_data,buffer_id->m_buffer_id);
    buffer_id->m_bm_data = bm_data;

    GBUFFER_DATA * pbuffer = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);
    pbuffer += buffer_id->m_buffer_id ;
    pbuffer->m_buffer_handle = newbufferhandle;
    pbuffer->m_size = buffer_size;
    pbuffer->m_usage = usage;
    pbuffer->m_lock_count = 0;
    pbuffer->m_refcount = 0;
    pbuffer->m_mapped_pointer = 0;

    //set shadow buffer if needed

    if(usage == G_MU_STATIC_READ ||
      usage == G_MU_STATIC_READ_DYNAMIC_WRITE||
      usage == G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
    {
        gim_create_common_buffer(buffer_managers,buffer_size,&pbuffer->m_shadow_buffer);
    }
    else
    {
    pbuffer->m_shadow_buffer.m_bm_data = 0;
        pbuffer->m_shadow_buffer.m_buffer_id = G_UINT_INFINITY;
    }
    return G_BUFFER_OP_SUCCESS;
}


GUINT32 gim_create_buffer_from_data(
  GBUFFER_MANAGER_DATA buffer_managers[],
    GUINT32 buffer_manager_id,
    const void * pdata,
    GUINT32 buffer_size,
    int usage,
    GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_MANAGER(buffer_managers,buffer_manager_id)

    GPTR newbufferhandle = bm_data->m_prototype->alloc_data_fn(pdata,buffer_size,usage);
    if(newbufferhandle==0) return G_BUFFER_OP_INVALID;

    GET_AVALIABLE_BUFFER_ID(bm_data,buffer_id->m_buffer_id);
    buffer_id->m_bm_data = bm_data;

    GBUFFER_DATA * pbuffer = GIM_DYNARRAY_POINTER(GBUFFER_DATA,bm_data->m_buffer_array);
    pbuffer += buffer_id->m_buffer_id ;
    pbuffer->m_buffer_handle = newbufferhandle;
    pbuffer->m_size = buffer_size;
    pbuffer->m_usage = usage;
    pbuffer->m_lock_count = 0;
    pbuffer->m_mapped_pointer = 0;
    pbuffer->m_refcount = 0;

    //set shadow buffer if needed

    if(usage == G_MU_STATIC_READ ||
      usage == G_MU_STATIC_READ_DYNAMIC_WRITE||
      usage == G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
    {
        gim_create_common_buffer_from_data(buffer_managers,pdata,buffer_size,&pbuffer->m_shadow_buffer);
    }
    else
    {
    pbuffer->m_shadow_buffer.m_bm_data = 0;
        pbuffer->m_shadow_buffer.m_buffer_id = G_UINT_INFINITY;
    }
    return G_BUFFER_OP_SUCCESS;
}

GUINT32 gim_create_common_buffer(GBUFFER_MANAGER_DATA buffer_managers[],
  GUINT32 buffer_size, GBUFFER_ID * buffer_id)
{
    return gim_create_buffer(buffer_managers,G_BUFFER_MANAGER_SYSTEM,buffer_size,G_MU_DYNAMIC_READ_WRITE,buffer_id);
}

GUINT32 gim_create_common_buffer_from_data(GBUFFER_MANAGER_DATA buffer_managers[],
    const void * pdata, GUINT32 buffer_size, GBUFFER_ID * buffer_id)
{
    return gim_create_buffer_from_data(buffer_managers,G_BUFFER_MANAGER_SYSTEM,pdata,buffer_size,G_MU_DYNAMIC_READ_WRITE,buffer_id);
}

GUINT32 gim_create_shared_buffer_from_data(GBUFFER_MANAGER_DATA buffer_managers[],
    const void * pdata, GUINT32 buffer_size, GBUFFER_ID * buffer_id)
{
    return gim_create_buffer_from_data(buffer_managers,G_BUFFER_MANAGER_SHARED,pdata,buffer_size,G_MU_DYNAMIC_READ_WRITE,buffer_id);
}

GINT32 gim_buffer_realloc(GBUFFER_ID * buffer_id,GUINT32 newsize)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count>0) return G_BUFFER_OP_INVALID;
    GPTR newhandle = buffer_id->m_bm_data->m_prototype->realloc_fn(
    pbuffer->m_buffer_handle,pbuffer->m_size,pbuffer->m_usage,newsize,pbuffer->m_usage);
    if(newhandle==0) return G_BUFFER_OP_INVALID;
    pbuffer->m_buffer_handle = newhandle;
    //realloc shadow buffer if any
    gim_buffer_realloc(&pbuffer->m_shadow_buffer,newsize);
    return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_buffer_add_ref(GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    pbuffer->m_refcount++;
    return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_buffer_free(GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count>0) return G_BUFFER_OP_INVALID;
    if(pbuffer->m_refcount>0) pbuffer->m_refcount--;
    if(pbuffer->m_refcount>0) return G_BUFFER_OP_STILLREFCOUNTED;

    buffer_id->m_bm_data->m_prototype->free_fn(
    pbuffer->m_buffer_handle,pbuffer->m_size);
    //destroy shadow buffer if needed
    gim_buffer_free(&pbuffer->m_shadow_buffer);
    // Obtain a free slot index for a new buffer
    GIM_DYNARRAY_PUSH_ITEM(GUINT32,bm_data->m_free_positions,buffer_id->m_buffer_id);
  pbuffer->m_buffer_handle = 0;
  pbuffer->m_size = 0;
  pbuffer->m_shadow_buffer.m_bm_data = 0;
  pbuffer->m_shadow_buffer.m_buffer_id = G_UINT_INFINITY;
    return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_lock_buffer(GBUFFER_ID * buffer_id,int access,char ** map_pointer)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count>0)
    {
        if(pbuffer->m_access!=access) return G_BUFFER_OP_INVALID;
        pbuffer->m_lock_count++;
        *map_pointer = pbuffer->m_mapped_pointer;
        return G_BUFFER_OP_SUCCESS;
    }

    pbuffer->m_access = access;

    GUINT32 result;
    if(pbuffer->m_usage==G_MU_STATIC_WRITE)
  {
    *map_pointer = 0;///no access
    return G_BUFFER_OP_INVALID;
  }
  else if(pbuffer->m_usage==G_MU_STATIC_READ)
  {
    if(pbuffer->m_access == G_MA_READ_ONLY)
    {
      result = gim_lock_buffer(&pbuffer->m_shadow_buffer,access,map_pointer);
      if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
      pbuffer->m_mapped_pointer = *map_pointer;
      pbuffer->m_lock_count++;
    }
    else
    {
      *map_pointer = 0;
      return G_BUFFER_OP_INVALID;
    }
  }
  else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE)
  {
    if(pbuffer->m_access == G_MA_READ_ONLY)
    {
      result = gim_lock_buffer(&pbuffer->m_shadow_buffer,access,map_pointer);
      if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
      pbuffer->m_mapped_pointer = *map_pointer;
      pbuffer->m_lock_count++;
    }
    else if(pbuffer->m_access == G_MA_WRITE_ONLY)
    {
        pbuffer->m_mapped_pointer = buffer_id->m_bm_data->m_prototype->lock_buffer_fn(
        pbuffer->m_buffer_handle,access);
            *map_pointer = pbuffer->m_mapped_pointer;
      pbuffer->m_lock_count++;
    }
    else if(pbuffer->m_access == G_MA_READ_WRITE)
    {
      *map_pointer = 0;
      return G_BUFFER_OP_INVALID;
    }
  }
  else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
  {
    result = gim_lock_buffer(&pbuffer->m_shadow_buffer,access,map_pointer);
        if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
        pbuffer->m_mapped_pointer = *map_pointer;
        pbuffer->m_lock_count++;
  }
  else if(pbuffer->m_usage==G_MU_STATIC_WRITE_DYNAMIC_READ)
  {
    if(pbuffer->m_access == G_MA_READ_ONLY)
    {
      pbuffer->m_mapped_pointer = buffer_id->m_bm_data->m_prototype->lock_buffer_fn(
        pbuffer->m_buffer_handle,access);
            *map_pointer = pbuffer->m_mapped_pointer;
      pbuffer->m_lock_count++;
    }
    else
    {
      *map_pointer = 0;
      return G_BUFFER_OP_INVALID;
    }
  }
  else if(pbuffer->m_usage==G_MU_DYNAMIC_READ_WRITE)
  {
    pbuffer->m_mapped_pointer = buffer_id->m_bm_data->m_prototype->lock_buffer_fn(
      pbuffer->m_buffer_handle,access);
        *map_pointer = pbuffer->m_mapped_pointer;
        pbuffer->m_lock_count++;
  }
    return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_unlock_buffer(GBUFFER_ID * buffer_id)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    if(pbuffer->m_lock_count==0) return G_BUFFER_OP_INVALID;

    if(pbuffer->m_lock_count>1)
    {
        pbuffer->m_lock_count--;
        return G_BUFFER_OP_SUCCESS;
    }


    GUINT32 result;
    if(pbuffer->m_usage==G_MU_STATIC_WRITE)
  {
    pbuffer->m_mapped_pointer = 0;
        pbuffer->m_lock_count=0;
        return G_BUFFER_OP_INVALID;
  }
  else if(pbuffer->m_usage==G_MU_STATIC_READ)
  {
    if(pbuffer->m_access == G_MA_READ_ONLY)
    {
      result = gim_unlock_buffer(&pbuffer->m_shadow_buffer);
      if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
      pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
    }
    else
    {
      pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
      return G_BUFFER_OP_INVALID;
    }
  }
  else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE)
  {
    if(pbuffer->m_access == G_MA_READ_ONLY)
    {
      result = gim_unlock_buffer(&pbuffer->m_shadow_buffer);
      if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
      pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
    }
    else if(pbuffer->m_access == G_MA_WRITE_ONLY)
    {
        buffer_id->m_bm_data->m_prototype->unlock_buffer_fn(
        pbuffer->m_buffer_handle);
            pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
    }
    else if(pbuffer->m_access == G_MA_READ_WRITE)
    {
      pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
      return G_BUFFER_OP_INVALID;
    }
  }
  else if(pbuffer->m_usage==G_MU_STATIC_READ_DYNAMIC_WRITE_COPY)
  {
      result = gim_unlock_buffer(&pbuffer->m_shadow_buffer);
        if(result!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;
        pbuffer->m_mapped_pointer = 0;
        pbuffer->m_lock_count=0;
      if(pbuffer->m_access == G_MA_WRITE_ONLY||pbuffer->m_access == G_MA_READ_WRITE)
    {
        gim_copy_buffers(&pbuffer->m_shadow_buffer,0,buffer_id,0,pbuffer->m_size);
    }
  }
  else if(pbuffer->m_usage==G_MU_STATIC_WRITE_DYNAMIC_READ)
  {
    if(pbuffer->m_access == G_MA_READ_ONLY)
    {
      buffer_id->m_bm_data->m_prototype->unlock_buffer_fn(
        pbuffer->m_buffer_handle);
            pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
    }
    else
    {
      pbuffer->m_mapped_pointer = 0;
      pbuffer->m_lock_count=0;
      return G_BUFFER_OP_INVALID;
    }
  }
  else if(pbuffer->m_usage==G_MU_DYNAMIC_READ_WRITE)
  {
    buffer_id->m_bm_data->m_prototype->unlock_buffer_fn(
      pbuffer->m_buffer_handle);
        pbuffer->m_mapped_pointer = 0;
        pbuffer->m_lock_count=0;
  }
  return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_get_buffer_size(GBUFFER_ID * buffer_id,GUINT32 * buffer_size)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    *buffer_size =  pbuffer->m_size;
    return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_get_buffer_is_locked(GBUFFER_ID * buffer_id,GUINT32 * lock_count)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    *lock_count =  pbuffer->m_lock_count;
    return G_BUFFER_OP_SUCCESS;
}


GINT32 gim_download_from_buffer(
        GBUFFER_ID * buffer_id,
    GUINT32 source_pos,
    void * destdata,
    GUINT32 copysize)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    buffer_id->m_bm_data->m_prototype->download_from_buffer_fn(
        pbuffer->m_buffer_handle,source_pos,destdata,copysize);
    return G_BUFFER_OP_SUCCESS;
}

GINT32  gim_upload_to_buffer(
    GBUFFER_ID * buffer_id,
    GUINT32 dest_pos,
    void * sourcedata,
    GUINT32 copysize)
{
    VALIDATE_BUFFER_ID_PT(buffer_id)
    buffer_id->m_bm_data->m_prototype->upload_to_buffer_fn(
        pbuffer->m_buffer_handle,dest_pos,sourcedata,copysize);
    return G_BUFFER_OP_SUCCESS;
}

GINT32  gim_copy_buffers(
    GBUFFER_ID * source_buffer_id,
    GUINT32 source_pos,
    GBUFFER_ID * dest_buffer_id,
    GUINT32 dest_pos,
    GUINT32 copysize)
{
    GBUFFER_MANAGER_DATA * bm_data1,* bm_data2;
    GBUFFER_DATA * pbuffer1, * pbuffer2;
    void * tempdata;
    if(_validate_buffer_id(source_buffer_id,&pbuffer1,&bm_data1)!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;

    if(_validate_buffer_id(dest_buffer_id,&pbuffer2,&bm_data2)!= G_BUFFER_OP_SUCCESS) return G_BUFFER_OP_INVALID;

    if((bm_data1->m_buffer_manager_id == bm_data2->m_buffer_manager_id)||
        (bm_data1->m_buffer_manager_id == G_BUFFER_MANAGER_SYSTEM && bm_data2->m_buffer_manager_id == G_BUFFER_MANAGER_SHARED)||
        (bm_data1->m_buffer_manager_id == G_BUFFER_MANAGER_SHARED && bm_data2->m_buffer_manager_id == G_BUFFER_MANAGER_SYSTEM)
    )
    {//smooth copy
        bm_data1->m_prototype->copy_buffers_fn(
      pbuffer1->m_buffer_handle,source_pos,pbuffer2->m_buffer_handle,dest_pos,copysize);
    }
    else if(bm_data1->m_buffer_manager_id == G_BUFFER_MANAGER_SYSTEM ||
    bm_data1->m_buffer_manager_id == G_BUFFER_MANAGER_SHARED)
    {
        //hard copy
        tempdata = (void *)pbuffer1->m_buffer_handle;
        //upload data
        bm_data2->m_prototype->upload_to_buffer_fn(
      pbuffer2->m_buffer_handle,dest_pos,tempdata,copysize);
    }
    else
    {
        //very hard copy
        void *tempdataA = gim_alloc(copysize);
        //download data
        bm_data1->m_prototype->download_from_buffer_fn(
      pbuffer1->m_buffer_handle,source_pos,tempdataA,copysize);

        //upload data
        bm_data2->m_prototype->upload_to_buffer_fn(
      pbuffer2->m_buffer_handle,dest_pos,tempdataA,copysize);
        //delete temp buffer
        gim_free(tempdataA,copysize);
    }
    return G_BUFFER_OP_SUCCESS;
}

GINT32 gim_buffer_array_lock(GBUFFER_ARRAY * array_data, int access)
{
    if(array_data->m_buffer_data != 0) return G_BUFFER_OP_SUCCESS;
    GINT32 result = gim_lock_buffer(&array_data->m_buffer_id,access,&array_data->m_buffer_data);
    if(result!= G_BUFFER_OP_SUCCESS) return result;
    array_data->m_buffer_data += array_data->m_byte_offset;
    return result;
}

GINT32 gim_buffer_array_unlock(GBUFFER_ARRAY * array_data)
{
    if(array_data->m_buffer_data == 0) return G_BUFFER_OP_SUCCESS;
    GINT32 result = gim_unlock_buffer(&array_data->m_buffer_id);
    if(result!= G_BUFFER_OP_SUCCESS) return result;
    array_data->m_buffer_data = 0;
    return result;
}

void gim_buffer_array_copy_ref(GBUFFER_ARRAY * source_data,GBUFFER_ARRAY  * dest_data)
{
    dest_data->m_buffer_id.m_buffer_id = source_data->m_buffer_id.m_buffer_id;
    dest_data->m_buffer_id.m_bm_data = source_data->m_buffer_id.m_bm_data;
    dest_data->m_buffer_data = 0;
    dest_data->m_byte_stride = source_data->m_byte_stride;
    dest_data->m_byte_offset = source_data->m_byte_offset;
    dest_data->m_element_count = source_data->m_element_count;
    gim_buffer_add_ref(&dest_data->m_buffer_id);
}

void gim_buffer_array_copy_value(GBUFFER_ARRAY * source_data,
  GBUFFER_MANAGER_DATA dest_buffer_managers[],GBUFFER_ARRAY  * dest_data,
  GUINT32 buffer_manager_id,int usage)
{
    //Create new buffer
    GUINT32 buffsize = source_data->m_element_count*source_data->m_byte_stride;
    gim_create_buffer(dest_buffer_managers,buffer_manager_id,buffsize,usage,&dest_data->m_buffer_id);

    //copy ref data
    dest_data->m_buffer_data = 0;
    dest_data->m_byte_stride = source_data->m_byte_stride;
    dest_data->m_byte_offset = 0;
    dest_data->m_element_count = source_data->m_element_count;
    gim_buffer_add_ref(&dest_data->m_buffer_id);
    //copy buffers
    gim_copy_buffers(&source_data->m_buffer_id,source_data->m_byte_offset,&dest_data->m_buffer_id,0,buffsize);
}
