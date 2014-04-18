/*************************************************************************
 *                                                                       *
 * Open Dynamics Engine, Copyright (C) 2001,2002 Russell L. Smith.       *
 * All rights reserved.  Email: russ@q12.org   Web: www.q12.org          *
 *                                                                       *
 * This library is free software; you can redistribute it and/or         *
 * modify it under the terms of EITHER:                                  *
 *   (1) The GNU Lesser General Public License as published by the Free  *
 *       Software Foundation; either version 2.1 of the License, or (at  *
 *       your option) any later version. The text of the GNU Lesser      *
 *       General Public License is included with this library in the     *
 *       file LICENSE.TXT.                                               *
 *   (2) The BSD-style license that is included with this library in     *
 *       the file LICENSE-BSD.TXT.                                       *
 *                                                                       *
 * This library is distributed in the hope that it will be useful,       *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the files    *
 * LICENSE.TXT and LICENSE-BSD.TXT for more details.                     *
 *                                                                       *
 *************************************************************************/

#ifndef _ODE_JOINT_GEARBOX_
#define _ODE_JOINT_GEARBOX_

#include "joint.h"

struct dxJointGearbox : public dxJoint
{
    dVector3 axis1, axis2;
    dReal ratio;        // gearbox ratio
    dReal erp;          // error reduction
    dReal cfm;          // constraint force mix in
    dxBody *refBody1;    // reference body for calculating gear angle 1
    dxBody *refBody2;    // reference body for calculating gear angle 2
    dReal cumulative_angle1;
    dReal cumulative_angle2;
    dQuaternion qrel1;   // initial relative rotation refBody1 -> body1
    dQuaternion qrel2;   // initial relative rotation refBody2 -> body2
    
    dxJointGearbox(dxWorld *w);

    virtual void getSureMaxInfo( SureMaxInfo* info );
    virtual void getInfo1( Info1* info );
    virtual void getInfo2( Info2* info );
    virtual dJointType type() const;
    virtual size_t size() const;
};

#endif
