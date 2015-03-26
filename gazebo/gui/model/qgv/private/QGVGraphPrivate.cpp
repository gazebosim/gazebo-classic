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

#include "QGVGraphPrivate.h"

QGVGraphPrivate::QGVGraphPrivate(Agraph_t *agraph)
{
  setGraph(agraph);
}

void QGVGraphPrivate::setGraph(Agraph_t *agraph)
{
  _graph = agraph;
}

Agraph_t* QGVGraphPrivate::graph() const
{
  return _graph;
}
