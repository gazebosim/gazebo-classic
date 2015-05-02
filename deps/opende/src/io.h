/*
   Copyright (c) 2013 Claude Lacoursiere

   This software is provided 'as-is', without any express or implied
   warranty. In no event will the authors be held liable for any damages
   arising from the use of this software.

   Permission is granted to anyone to use this software for any purpose,
   including commercial applications, and to alter it and redistribute it
   freely, subject to the following restrictions:

   1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.

   2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.

   3. This notice may not be removed or altered from any source
   distribution.

reference:
   hdf5-based BPMD database: https://grasp.robotics.cs.rpi.edu/bpmd/
*/

#include "gazebo/gazebo_config.h"
#ifdef HDF5_INSTRUMENT


#ifndef H5_IO
#define H5_IO
#include <string>
#include <ode/ode.h>
#include <unordered_map>
class h5dump {
  private:
  std::string file;           // data will be accumulated in this file
  dxWorld * world;
  dReal step_size;
  size_t nbodies;             // number of bodies in current frame
  void **body_data;           // place holder to save and restore userdata in bodies
  int  * body_labels;         // body indices starting from 1 for the dump
  dReal * x;                  // positions
  dReal * q;                  // quaternions
  dReal * v;                  // linear and angular velocities
  dReal * f;                  // forces and torques
  dReal * m;                  // mass
  dReal  *M;                  // inertia tensor
  int   *id;                  // body labels to match constraints
  int   *state;               // body labels to match constraints
  std::unordered_map< void*, int> tags; // tags are not guaranteed to be constant in
  // so we need to user pointers to do that.

  // data for constraints
  size_t nconstraints;          // number of constraints in current frame
  size_t  nrows;                // total number of rows
  size_t  *rows;                // number of rows in one constraint
  dReal * g;                    // constraint violation
  dReal * gt;                   // velocity for rheonomic constraints
  dReal * bounds;               // lower bound for bounded constraints
  int *  constraint_pairs;      // pairs of bodies bound by constraints
  dReal *G;                     // The Jacobian packed in a matrix with nrows and 12 columns.  This is row major
  bool *holonomic;              // Whether a constraint is holonomic or not.
  const size_t ldg;             // the leading dimension for G

  // data for contacts
  size_t ncontacts;             // number of contacts in current frame
  dReal * points;               // contact points
  dReal * normals;              // contact normals
  dReal * gaps;                 // gap distance, -1*penetration depth
  dReal * mu_s;                 // static friction coefficient
  dReal * mu_k;                 // kinetic friction coefficient
  int *  contact_pairs;         // pairs of bodies bound by contacts

  // The internal API
  // Body manipulation and extraction
  void allocate_bodies();
  void free_bodies();
  void pack_bodies();
  void dump_bodies();

  // Constraint manipulation and extraction
  void allocate_constraints();
  void free_constraints();
  void pack_constraints();
  void dump_constraints();

  // Contact manipulation and extraction
  void allocate_contacts();
  void free_contacts();
  void pack_contacts();
  void dump_contacts();
  void dump( bool first );

  public:
  h5dump(const std::string & filename, dxWorld *w, dReal stepsize, bool first, size_t l = 12) :
    file(filename), world(w), step_size(stepsize), ldg(l) {
    dump( first );
  }
};

#endif // H5_IO
#endif // HDF5_INSTRUMENT
