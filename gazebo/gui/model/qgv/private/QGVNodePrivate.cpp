#include "QGVNodePrivate.h"

QGVNodePrivate::QGVNodePrivate(Agnode_t *agnode)
{
    setNode(agnode);
}

void QGVNodePrivate::setNode(Agnode_t *agnode)
{
  _node = agnode;
}

Agnode_t* QGVNodePrivate::node() const
{
  return _node;
}
