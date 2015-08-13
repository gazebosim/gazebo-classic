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

#ifndef QGVEDGEPRIVATE_H
#define QGVEDGEPRIVATE_H

#include <cgraph.h>

class QGVEdgePrivate
{
  public:
    explicit QGVEdgePrivate(Agedge_t *edge = NULL);

    void setEdge(Agedge_t *edge);
    Agedge_t* edge() const;

  private:
    Agedge_t* _edge;
};

#endif
