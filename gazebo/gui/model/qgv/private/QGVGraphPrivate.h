#ifndef QGVGRAPHPRIVATE_H
#define QGVGRAPHPRIVATE_H

#include <cgraph.h>

class QGVGraphPrivate
{
	public:
		QGVGraphPrivate(Agraph_t *graph = NULL);

		void setGraph(Agraph_t *graph);
		Agraph_t* graph() const;

	private:
		Agraph_t* _graph;
};

#endif // QGVGRAPHPRIVATE_H
