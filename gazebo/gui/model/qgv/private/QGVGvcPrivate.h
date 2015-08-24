/***************************************************************
QGVCore
Copyright (c) 2014, Bergont Nicolas, All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 3.0 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library.
***************************************************************/

#ifndef QGVGVCPRIVATE_H
#define QGVGVCPRIVATE_H

#include <gvc.h>

class QGVGvcPrivate
{
  public:
    explicit QGVGvcPrivate(GVC_t *context = NULL);

    void setContext(GVC_t *context);
    GVC_t* context() const;

    // operators to implicit cast from QGVGvcPrivate* into
    // GVC_t* seems not to work,
    // because of typedef GVC_t
//    inline operator const GVC_t* () const
//    {
//      return const_cast<VC_t*>(context());
//    }

//    inline operator struct GVC_t* ()
//    {
//      return context();
//    }

  private:
    GVC_t* _context;
};

#endif
