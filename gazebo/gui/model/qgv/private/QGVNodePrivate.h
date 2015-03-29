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

#ifndef QGVNODEPRIVATE_H
#define QGVNODEPRIVATE_H

#include <cgraph.h>

class QGVNodePrivate
{
  public:
    explicit QGVNodePrivate(Agnode_t *node = NULL);

    void setNode(Agnode_t *node);
    Agnode_t* node() const;

  private:
    Agnode_t* _node;
};

#endif
