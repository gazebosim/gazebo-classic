
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


#include "GIMPACT/gim_boxpruning.h"



//! Allocate memory for all aabb set.
void gim_aabbset_alloc(GIM_AABB_SET * aabbset, GUINT32 count)
{
    aabbset->m_count = count;
    aabbset->m_boxes = (aabb3f *)gim_alloc(sizeof(aabb3f)*count);

    if(count<GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES)
    {
        aabbset->m_maxcoords = 0;
        aabbset->m_sorted_mincoords = 0;
    }
    else
    {
        aabbset->m_maxcoords = (GUINT32 *)gim_alloc(sizeof(GUINT32)*aabbset->m_count );
        aabbset->m_sorted_mincoords = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN)*aabbset->m_count);
    }
    aabbset->m_shared = 0;
    INVALIDATE_AABB(aabbset->m_global_bound);
}

//! Destroys the aabb set.
void gim_aabbset_destroy(GIM_AABB_SET * aabbset)
{
    aabbset->m_count = 0;
    if(aabbset->m_shared==0)
    {
        gim_free(aabbset->m_boxes,0);
        gim_free(aabbset->m_maxcoords,0);
        gim_free(aabbset->m_sorted_mincoords,0);
    }
    aabbset->m_boxes = 0;
    aabbset->m_sorted_mincoords = 0;
    aabbset->m_maxcoords = 0;
}

void gim_aabbset_calc_global_bound(GIM_AABB_SET * aabbset)
{
    aabb3f * paabb = aabbset->m_boxes;
    aabb3f * globalbox = &aabbset->m_global_bound;
    AABB_COPY((*globalbox),(*paabb));

    GUINT32 count = aabbset->m_count-1;
    paabb++;
    while(count)
    {
        MERGEBOXES(*globalbox,*paabb)
        paabb++;
        count--;
    }
}


//! Sorts the boxes for box prunning.
/*!
1) find the integer representation of the aabb coords
2) Sorts the min coords
3) Calcs the global bound
\pre aabbset must be allocated. And the boxes must be already set.
\param aabbset
\param calc_global_bound If 1 , calcs the global bound
\post If aabbset->m_sorted_mincoords == 0, then it allocs the sorted coordinates
*/
void gim_aabbset_sort(GIM_AABB_SET * aabbset, char calc_global_bound)
{
    if(aabbset->m_sorted_mincoords == 0)
    {//allocate
        aabbset->m_maxcoords = (GUINT32 *)gim_alloc(sizeof(GUINT32)*aabbset->m_count );
        aabbset->m_sorted_mincoords = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN)*aabbset->m_count);
    }

    GUINT32 i, count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    GUINT32 * maxcoords = aabbset->m_maxcoords;
    GIM_RSORT_TOKEN * sorted_tokens = aabbset->m_sorted_mincoords;

    if(count<860)//Calibrated on a Pentium IV
    {
        //Sort by quick sort
            //Calculate keys
        for(i=0;i<count;i++)
        {
            GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(paabb[i].maxX,paabb[i].maxZ,maxcoords[i]);
            GIM_CONVERT_VEC3F_GUINT_XZ(paabb[i].minX,paabb[i].minZ,sorted_tokens[i].m_key);
            sorted_tokens[i].m_value = i;
        }
        GIM_QUICK_SORT_ARRAY(GIM_RSORT_TOKEN , sorted_tokens, count, RSORT_TOKEN_COMPARATOR,GIM_DEF_EXCHANGE_MACRO);
    }
    else
    {
        //Sort by radix sort
        GIM_RSORT_TOKEN * unsorted = (GIM_RSORT_TOKEN *)gim_alloc(sizeof(GIM_RSORT_TOKEN )*count);
        //Calculate keys
        for(i=0;i<count;i++)
        {
            GIM_CONVERT_VEC3F_GUINT_XZ_UPPER(paabb[i].maxX,paabb[i].maxZ,maxcoords[i]);
            GIM_CONVERT_VEC3F_GUINT_XZ(paabb[i].minX,paabb[i].minZ,unsorted[i].m_key);
            unsorted[i].m_value = i;
        }
        GIM_RADIX_SORT_RTOKENS(unsorted,sorted_tokens,count);
        gim_free(unsorted,0);
    }

    if(calc_global_bound) gim_aabbset_calc_global_bound(aabbset);
}

//utility macros

/*#define PUSH_PAIR(i,j,pairset)\
{\
    GIM_PAIR _pair={i,j};\
    GIM_DYNARRAY_PUSH_ITEM(GIM_PAIR,pairset,_pair);\
}*/

#define PUSH_PAIR(i,j,pairset)\
{\
    GIM_DYNARRAY_PUSH_EMPTY(GIM_PAIR,pairset);\
    GIM_PAIR * _pair = GIM_DYNARRAY_POINTER(GIM_PAIR,pairset) + (pairset).m_size - 1;\
    _pair->m_index1 = i;\
    _pair->m_index2 = j;\
}

#define PUSH_PAIR_INV(i,j,pairset)\
{\
    GIM_DYNARRAY_PUSH_EMPTY(GIM_PAIR,pairset);\
    GIM_PAIR * _pair = GIM_DYNARRAY_POINTER(GIM_PAIR,pairset) + (pairset).m_size - 1;\
    _pair->m_index1 = j;\
    _pair->m_index2 = i;\
}

#define FIND_OVERLAPPING_FOWARD(\
 curr_index,\
 test_count,\
 test_aabb,\
 max_coord_uint,\
 sorted_tokens,\
 aabbarray,\
 pairset,\
 push_pair_macro)\
{\
    GUINT32 _i = test_count;\
    char _intersected;\
    GIM_RSORT_TOKEN * _psorted_tokens = sorted_tokens;\
    while(_i>0 && max_coord_uint >= _psorted_tokens->m_key)\
    {\
        AABBCOLLISION(_intersected,test_aabb,aabbarray[_psorted_tokens->m_value]);\
      if(_intersected)\
      {\
          push_pair_macro(curr_index, _psorted_tokens->m_value,pairset);\
      }\
      _psorted_tokens++;\
      _i--;\
    }\
}

//! log(N) Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
\pre aabbset must be allocated and sorted, the boxes must be already set.
\param aabbset Must be sorted. Global bound isn't required
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_self_intersections_sorted(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
{
    collision_pairs->m_size = 0;
    GUINT32 count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    GUINT32 * maxcoords = aabbset->m_maxcoords;
    GIM_RSORT_TOKEN * sorted_tokens = aabbset->m_sorted_mincoords;
    aabb3f test_aabb;
    while(count>1)
    {
        ///current cache variables
        GUINT32  curr_index = sorted_tokens->m_value;
        GUINT32 max_coord_uint = maxcoords[curr_index];
        AABB_COPY(test_aabb,paabb[curr_index]);

        ///next pairs
        sorted_tokens++;
        count--;
      FIND_OVERLAPPING_FOWARD( curr_index, count, test_aabb, max_coord_uint, sorted_tokens , paabb, (*collision_pairs),PUSH_PAIR);
    }
}

//! NxN Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
\pre aabbset must be allocated, the boxes must be already set.
\param aabbset Global bound isn't required. Doen't need to be sorted.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_self_intersections_brute_force(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
{
    collision_pairs->m_size = 0;
    GUINT32 i,j;
    GUINT32 count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    char intersected;
    for (i=0;i< count-1 ;i++ )
    {
        for (j=i+1;j<count ;j++ )
        {
            AABBCOLLISION(intersected,paabb[i],paabb[j]);
            if(intersected)
            {
                PUSH_PAIR(i,j,(*collision_pairs));
            }
        }
    }
}

//! log(N) Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections_sorted(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
{
    char intersected;
    collision_pairs->m_size = 0;

    AABBCOLLISION(intersected,aabbset1->m_global_bound,aabbset2->m_global_bound);
    if(intersected == 0) return;

    GUINT32 count1 = aabbset1->m_count;
    aabb3f * paabb1 = aabbset1->m_boxes;
    GUINT32 * maxcoords1 = aabbset1->m_maxcoords;
    GIM_RSORT_TOKEN * sorted_tokens1 = aabbset1->m_sorted_mincoords;

    GUINT32 count2 = aabbset2->m_count;
    aabb3f * paabb2 = aabbset2->m_boxes;
    GUINT32 * maxcoords2 = aabbset2->m_maxcoords;
    GIM_RSORT_TOKEN * sorted_tokens2 = aabbset2->m_sorted_mincoords;

    GUINT32  curr_index;

    GUINT32 max_coord_uint;
    aabb3f test_aabb;

    //Classify boxes
    //Find  Set intersection
    aabb3f int_abbb;
    BOXINTERSECTION(aabbset1->m_global_bound,aabbset2->m_global_bound, int_abbb);

    //Clasify set 1
    GIM_RSORT_TOKEN * classified_tokens1 = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN)*count1);
    GUINT32 i,classified_count1 = 0,classified_count2 = 0;


    for (i=0;i<count1;i++ )
    {
        curr_index = sorted_tokens1[i].m_value;
        AABBCOLLISION(intersected,paabb1[curr_index],int_abbb);
      if(intersected)
      {
          classified_tokens1[classified_count1] = sorted_tokens1[i];
          classified_count1++;
      }
    }

    if(classified_count1==0)
    {
        gim_free(classified_tokens1 ,0);
        return; // no pairs
    }

    //Clasify set 2
    GIM_RSORT_TOKEN * classified_tokens2 = (GIM_RSORT_TOKEN *) gim_alloc(sizeof(GIM_RSORT_TOKEN)*count2);

    for (i=0;i<count2;i++ )
    {
        curr_index = sorted_tokens2[i].m_value;
        AABBCOLLISION(intersected,paabb2[curr_index],int_abbb);
      if(intersected)
      {
          classified_tokens2[classified_count2] = sorted_tokens2[i];
          classified_count2++;
      }
    }

    if(classified_count2==0)
    {
        gim_free(classified_tokens1 ,0);
        gim_free(classified_tokens2 ,0);
        return; // no pairs
    }

    sorted_tokens1 = classified_tokens1;
    sorted_tokens2 = classified_tokens2;

    while(classified_count1>0&&classified_count2>0)
    {
        if(sorted_tokens1->m_key <= sorted_tokens2->m_key)
        {
            ///current cache variables
            curr_index = sorted_tokens1->m_value;
            max_coord_uint = maxcoords1[curr_index];
            AABB_COPY(test_aabb,paabb1[curr_index]);
            ///next pairs
            sorted_tokens1++;
            classified_count1--;
            FIND_OVERLAPPING_FOWARD( curr_index, classified_count2, test_aabb, max_coord_uint, sorted_tokens2 , paabb2, (*collision_pairs), PUSH_PAIR);
        }
        else ///Switch test
        {
            ///current cache variables
            curr_index = sorted_tokens2->m_value;
            max_coord_uint = maxcoords2[curr_index];
            AABB_COPY(test_aabb,paabb2[curr_index]);
            ///next pairs
            sorted_tokens2++;
            classified_count2--;
            FIND_OVERLAPPING_FOWARD( curr_index, classified_count1, test_aabb, max_coord_uint, sorted_tokens1 , paabb1, (*collision_pairs), PUSH_PAIR_INV );
        }
    }
    gim_free(classified_tokens1 ,0);
    gim_free(classified_tokens2 ,0);
}

//! NxM Bipartite box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and sorted, the boxes must be already set.
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections_brute_force(GIM_AABB_SET * aabbset1,GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
{
    char intersected;
    collision_pairs->m_size = 0;
    AABBCOLLISION(intersected,aabbset1->m_global_bound,aabbset2->m_global_bound);
    if(intersected == 0) return;

    aabb3f int_abbb;
    //Find  Set intersection
    BOXINTERSECTION(aabbset1->m_global_bound,aabbset2->m_global_bound, int_abbb);
    //Clasify set 1
    GUINT32 i,j;
    GUINT32 classified_count = 0;

    GUINT32 count = aabbset1->m_count;
    aabb3f * paabb1 = aabbset1->m_boxes;
    aabb3f * paabb2 = aabbset2->m_boxes;

    GUINT32 * classified = (GUINT32 *) gim_alloc(sizeof(GUINT32)*count);

    for (i=0;i<count;i++ )
    {
        AABBCOLLISION(intersected,paabb1[i],int_abbb);
      if(intersected)
      {
          classified[classified_count] = i;
          classified_count++;
      }
    }

    if(classified_count==0)
    {
        gim_free(classified,0);
        return; // no pairs
    }

    //intesect set2
    count = aabbset2->m_count;
    for (i=0;i<count;i++)
    {
        AABBCOLLISION(intersected,paabb2[i],int_abbb);
        if(intersected)
        {
            for (j=0;j<classified_count;j++)
            {
                AABBCOLLISION(intersected,paabb2[i],paabb1[classified[j]]);
                if(intersected)
                {
                    PUSH_PAIR(classified[j],i,(*collision_pairs));
                }
            }
        }
    }
    gim_free(classified,0);
}


//! Initalizes the set. Sort Boxes if needed.
/*!
\pre aabbset must be allocated. And the boxes must be already set.
\post If the set has less of GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES boxes, only calcs the global box,
 else it Sorts the entire set( Only applicable for large sets)
*/
void gim_aabbset_update(GIM_AABB_SET * aabbset)
{
    if(aabbset->m_count < GIM_MIN_SORTED_BIPARTITE_PRUNING_BOXES)
    {//Brute force approach
        gim_aabbset_calc_global_bound(aabbset);
    }
    else
    {//Sorted force approach
        gim_aabbset_sort(aabbset,1);
    }
}

//! Complete box pruning. Returns a list of overlapping pairs of boxes, each box of the pair belongs to the same set.
/*!
This function sorts the set and then it calls to gim_aabbset_self_intersections_brute_force or gim_aabbset_self_intersections_sorted.

\param aabbset Set of boxes. Sorting isn't required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
\pre aabbset must be allocated and initialized.
\post If aabbset->m_count >= GIM_MIN_SORTED_PRUNING_BOXES, then it calls to gim_aabbset_sort and then to gim_aabbset_self_intersections_sorted.
*/
void gim_aabbset_self_intersections(GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collision_pairs)
{
    if(aabbset->m_count < GIM_MIN_SORTED_PRUNING_BOXES)
    {//Brute force approach
        gim_aabbset_self_intersections_brute_force(aabbset,collision_pairs);
    }
    else
    {//Sorted force approach
        gim_aabbset_sort(aabbset,0);
        gim_aabbset_self_intersections_sorted(aabbset,collision_pairs);
    }
}

//! Collides two sets. Returns a list of overlapping pairs of boxes, each box of the pair belongs to a different set.
/*!
\pre aabbset1 and aabbset2 must be allocated and updated. See .
\param aabbset1 Must be sorted, Global bound is required.
\param aabbset2 Must be sorted, Global bound is required.
\param collision_pairs Array of GIM_PAIR elements. Must be initialized before (Reserve size ~ 100)
*/
void gim_aabbset_bipartite_intersections(GIM_AABB_SET * aabbset1, GIM_AABB_SET * aabbset2, GDYNAMIC_ARRAY * collision_pairs)
{
    if(aabbset1->m_sorted_mincoords == 0||aabbset2->m_sorted_mincoords == 0)
    {//Brute force approach
        gim_aabbset_bipartite_intersections_brute_force(aabbset1,aabbset2,collision_pairs);
    }
    else
    {//Sorted force approach
        gim_aabbset_bipartite_intersections_sorted(aabbset1,aabbset2,collision_pairs);
    }
}

void gim_aabbset_box_collision(aabb3f *test_aabb, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    char intersected;
    AABBCOLLISION(intersected,aabbset->m_global_bound,(*test_aabb));
    if(intersected == 0) return;

    GUINT32 i;
    GUINT32 count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;
    aabb3f _testaabb;
    AABB_COPY(_testaabb,*test_aabb);

    for (i=0;i< count;i++ )
    {
        AABBCOLLISION(intersected,paabb[i],_testaabb);
        if(intersected)
        {
            GIM_DYNARRAY_PUSH_ITEM(GUINT32,(*collided),i);
        }
    }
}

void gim_aabbset_ray_collision(vec3f vorigin,vec3f vdir, GREAL tmax, GIM_AABB_SET * aabbset, GDYNAMIC_ARRAY * collided)
{
    collided->m_size = 0;
    char intersected;
    GREAL tparam = 0;
    (void)tparam;
    BOX_INTERSECTS_RAY(aabbset->m_global_bound, vorigin, vdir, tparam, tmax,intersected);
    if(intersected==0) return;

    GUINT32 i;
    GUINT32 count = aabbset->m_count;
    aabb3f * paabb = aabbset->m_boxes;

    for (i=0;i< count;i++ )
    {
        BOX_INTERSECTS_RAY(paabb[i], vorigin, vdir, tparam, tmax,intersected);
        if(intersected)
        {
            GIM_DYNARRAY_PUSH_ITEM(GUINT32,(*collided),i);
        }
    }
}
