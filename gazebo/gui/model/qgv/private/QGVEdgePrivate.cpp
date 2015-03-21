#include "QGVEdgePrivate.h"

QGVEdgePrivate::QGVEdgePrivate(Agedge_t *_agedge)
{
	setEdge(_agedge);
}

void QGVEdgePrivate::setEdge(Agedge_t *_agedge)
{
	_edge = _agedge;
}

Agedge_t* QGVEdgePrivate::edge() const
{
	return _edge;
}
