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
